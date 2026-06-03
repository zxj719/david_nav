#!/usr/bin/env python3
"""WebSocket client connecting to AI team's arm-tool WS server.

2026-06-02: AI team's actual architecture is WS (not the earlier HTTP POST
mini-spec). They listen on :8103, we connect actively. This client implements
the 6 atomic tools they wired (grab_bottle / grab_object / place_object /
cancel / get_arm_status / arm_home) and maps PickFromCam outcomes to their
11-code error_code schema with suggested_action.

Wire protocol (their doc 2026-06-02):

Request (their server -> us):
{
  "tool": "grab_bottle" | "grab_object" | "place_object" | "cancel" |
           "get_arm_status" | "arm_home",
  "request_id": "arm_<uuid12>",
  "protocol_version": "1.0",
  "args": {...},
  "issued_at": "..."
}

Response (us -> their server) on success:
{
  "request_id": "...",
  "status": "done",
  "duration_sec": float,
  "metadata": {
    "which_used": "left" | "right",
    "grasp_effort": int,
    "holding_object": bool,
    "object_class": str | null,
    "final_pose": "home" | "lift" | "unknown",
  }
}

On error:
{
  "request_id": "...",
  "status": "error" | "cancelled",
  "duration_sec": float,
  "metadata": {
    "error_code": "target_not_found" | "out_of_reach" | "out_of_workspace" |
                  "ik_failed" | "planning_failed" | "motion_failed" |
                  "collision" | "grasp_empty" | "grasp_dropped" |
                  "hardware_fault" | "cancelled",
    "error_msg": str,
    "suggested_action": str (optional, transparent passthrough to LLM)
  }
}

Health/handshake: server pings us via {"type": "ping"} every 30s, we respond
{"type": "pong"}. websocket-client also does its own low-level WS ping; we
handle both.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
import websocket  # websocket-client

from david_grasp.pick_from_cam import PickFromCam, CLOSE_SETTLE_SEC


# Defaults; overridable via CLI args / env / settings.yaml later.
DEFAULT_AI_HOST = '172.16.22.205'
DEFAULT_AI_PORT = 8103
DEFAULT_SIDE = 'left'   # right is out for thumb repair; flip when fixed.
PROTOCOL_VERSION = '1.0'

# Reach gate (per 2026-06-02 7-run test data):
# bottle.x in arm_base > 0.73 m -> IK reliably fails. Reject up-front so the
# LLM can ask nav to roll forward.
MAX_BOTTLE_X = 0.73

# Workspace gate: bottle.z in left arm base must be > 0 (bottle on left arm's
# outside / midline-left). Below this -> arm would cross body.
# We tolerate 2 cm into the midline; physically that's borderline-OK.
LEFT_MIN_BOTTLE_Z = -0.02

# Left arm home pose in URDF radians (= SDK radians since joint_signs=[1]*7).
# Unified home / safe-grasp pose for left arm (URDF rad = SDK rad since joint_signs=[1]*7).
# Per user 2026-06-02: [145, 90, -90, 140, 90, 0, 0] deg.
LEFT_HOME_RAD = [
    math.radians(145.0), math.radians(90.0), math.radians(-90.0),
    math.radians(140.0), math.radians(90.0), math.radians(0.0),
    math.radians(0.0),
]
# Unified home / safe-grasp pose for right arm.
# Per user 2026-06-02: [-55, 85, 100, 140, -90, 5, 0] deg.
RIGHT_HOME_RAD = [
    math.radians(-55.0), math.radians(85.0), math.radians(100.0),
    math.radians(140.0), math.radians(-90.0), math.radians(5.0),
    math.radians(0.0),
]

# Handover pose for left arm -- AI team triggers this when user asks
# "把水递给我" (give me the water). Per user 2026-06-03 (revised):
# [60, 90, -90, 70, 90, 0, 0] deg (was [80,_,_,85,_,_,_] on 2026-06-02).
LEFT_HANDOVER_RAD = [
    math.radians(60.0), math.radians(90.0), math.radians(-90.0),
    math.radians(70.0), math.radians(90.0), math.radians(0.0),
    math.radians(0.0),
]


def _err(code, msg, suggested_action=None):
    meta = {'error_code': code, 'error_msg': msg}
    if suggested_action:
        meta['suggested_action'] = suggested_action
    return {'status': 'error', 'metadata': meta}


class ArmGraspWSClient(PickFromCam):
    def __init__(self, side, ai_host, ai_port):
        super().__init__(side=side)
        self._ai_host = ai_host
        self._ai_port = ai_port
        self._stop_flag = False
        self._busy_lock = threading.Lock()
        self._cancel_flag = False

        # Holding state per arm (this client only drives one arm at a time
        # but we report status for both so LLM has full picture).
        self._holding = {
            'left':  {'holding_object': False, 'object_class': None, 'pose': 'unknown'},
            'right': {'holding_object': False, 'object_class': None, 'pose': 'unknown'},
        }

        self.get_logger().info(
            f'ArmGraspWSClient init: side={side}, '
            f'target WS server = ws://{ai_host}:{ai_port}')

    # ---- WS thread: connect loop with auto-reconnect ----------------------
    def run_ws_client(self):
        url = f'ws://{self._ai_host}:{self._ai_port}'
        backoff = 1.0
        while not self._stop_flag:
            try:
                self.get_logger().info(f'connecting to {url}')
                ws = websocket.WebSocketApp(
                    url,
                    on_open=self._on_ws_open,
                    on_message=self._on_ws_message,
                    on_error=self._on_ws_error,
                    on_close=self._on_ws_close,
                )
                # ping_interval/timeout: WS-level (frame-level) ping handled by lib.
                # AI team also uses app-level {"type":"ping"} which we handle below.
                ws.run_forever(ping_interval=30, ping_timeout=10)
                backoff = 1.0  # reset on clean close
            except Exception as e:
                self.get_logger().warn(f'WS connect error: {e}')
            if self._stop_flag:
                break
            self.get_logger().info(f'reconnecting in {backoff:.1f}s')
            time.sleep(backoff)
            backoff = min(backoff * 2, 30.0)

    def _on_ws_open(self, ws):
        self.get_logger().info('=== WS connected to AI arm tool server ===')

    def _on_ws_close(self, ws, code, reason):
        self.get_logger().warn(f'WS closed: code={code} reason={reason}')

    def _on_ws_error(self, ws, error):
        self.get_logger().warn(f'WS error: {type(error).__name__}: {error}')

    def _on_ws_message(self, ws, raw):
        # Spawn a worker thread so on_message returns quickly and WS recv
        # loop is not blocked by a 15s grasp.
        threading.Thread(
            target=self._handle_request,
            args=(ws, raw),
            daemon=True,
        ).start()

    # ---- Request handler --------------------------------------------------
    def _handle_request(self, ws, raw):
        try:
            req = json.loads(raw)
        except Exception as e:
            self.get_logger().warn(f'bad JSON: {e}')
            return

        # AI team uses app-level {"type":"ping"} every 30s
        if req.get('type') == 'ping':
            try:
                ws.send(json.dumps({'type': 'pong'}))
            except Exception as e:
                self.get_logger().warn(f'pong send failed: {e}')
            return

        tool = req.get('tool')
        request_id = req.get('request_id')
        args = req.get('args') or {}
        protocol_version = req.get('protocol_version', '1.0')

        if not request_id:
            self.get_logger().warn(f'request missing request_id; raw={raw[:200]}')
            return

        if protocol_version != PROTOCOL_VERSION:
            self.get_logger().warn(
                f'protocol_version mismatch: got {protocol_version}, '
                f'expected {PROTOCOL_VERSION}; parsing anyway per AI team policy')

        self.get_logger().info(f'>>> {request_id}: tool={tool} args={args}')

        t0 = time.time()
        try:
            if tool == 'grab_bottle':
                resp = self._tool_grab(args, object_class='bottle')
            elif tool == 'grab_object':
                object_id = args.get('object_id', 'bottle')
                if object_id != 'bottle':
                    resp = _err(
                        'target_not_found',
                        f'object_id={object_id!r} not supported (only "bottle" for now)',
                        'use grab_bottle, or wait for multi-class support')
                else:
                    resp = self._tool_grab(args, object_class='bottle')
            elif tool == 'place_object':
                resp = _err(
                    'planning_failed',
                    'place_object not implemented in MVP',
                    'demo path uses grab + nav only for now')
            elif tool == 'cancel':
                self._cancel_flag = True
                resp = {'status': 'cancelled', 'metadata': {}}
            elif tool == 'get_arm_status':
                resp = self._tool_get_arm_status()
            elif tool == 'arm_home':
                resp = self._tool_arm_home(args)
            elif tool == 'handover_water':
                resp = self._tool_handover_water(args)
            else:
                resp = _err('hardware_fault',
                            f'unknown tool {tool!r}',
                            'check the tool name; supported: '
                            'grab_bottle / grab_object / place_object / '
                            'cancel / get_arm_status / arm_home / '
                            'handover_water')
        except Exception as e:
            self.get_logger().error(
                f'uncaught exception in tool {tool}: {type(e).__name__}: {e}')
            resp = _err('hardware_fault',
                        f'{type(e).__name__}: {e}',
                        None)

        resp['request_id'] = request_id
        resp['duration_sec'] = round(time.time() - t0, 2)

        try:
            ws.send(json.dumps(resp, ensure_ascii=False))
            status = resp.get('status', '?')
            err_code = resp.get('metadata', {}).get('error_code')
            self.get_logger().info(
                f'<<< {request_id}: status={status} '
                f'{("err=" + err_code) if err_code else ""} '
                f'duration={resp["duration_sec"]}s')
        except Exception as e:
            self.get_logger().error(f'send response failed: {e}')

    # ---- Tool: grab_bottle / grab_object ----------------------------------
    def _tool_grab(self, args, object_class):
        if not self._busy_lock.acquire(blocking=False):
            return _err('hardware_fault',
                        'another action already in flight',
                        'wait for the current action to finish')
        try:
            which = args.get('which', 'auto')
            if which == 'auto':
                which = self.side  # MVP: only configured side is live
            if which != self.side:
                return _err('hardware_fault',
                            f'this client only drives side={self.side}; '
                            f'requested which={which}',
                            'use which="auto" or align with deployed side')

            # Reset session caches + cancel flag
            self._cached_bottle_arm = None
            self._cached_hover_joints = None
            self._cancel_flag = False

            # Pre-flight: bottle detection
            if self._last_point is None:
                return _err('target_not_found',
                            'no PointStamped received on /yolo/bottle_center',
                            'verify B vision pipeline is up')

            # Pre-flight: workspace + reach gates (BEFORE running stage 1 IK)
            # The detection cb already stored the latest msg; transform now
            # to check reach. _get_bottle_arm will be called again in hover()
            # but doing a fresh transform here gives early reject + clean code.
            pt = self._last_point
            b = self.cam_to_arm_base(pt)
            if b is None:
                return _err('target_not_found',
                            f'TF camera -> {self.plan_frame} failed',
                            'check robot_state_publisher and TF tree')
            if self.side == 'left' and b[2] < LEFT_MIN_BOTTLE_Z:
                return _err('out_of_workspace',
                            f'bottle.z={b[2]:+.3f} m in left arm base < '
                            f'{LEFT_MIN_BOTTLE_Z} -- bottle on/past body midline',
                            'let nav reposition robot ~10 cm to the right '
                            '(brings bottle further to robot left), or use right arm')
            if b[0] > MAX_BOTTLE_X:
                return _err('out_of_reach',
                            f'bottle.x={b[0]:.3f} m > reach limit {MAX_BOTTLE_X} m',
                            'let nav reposition robot ~10 cm forward')

            # Stage 1: open + hover to grasp pose
            self.get_logger().info('STAGE 1/3: open + move to grasp pose')
            ok = self.hover(mode='hover', fresh=True)
            if self._cancel_flag:
                return {'status': 'cancelled', 'metadata': {'stage': 'after_hover'}}
            if not ok:
                # _cached_bottle_arm being None means detection/TF/workspace failed;
                # otherwise it's an IK/motion failure
                if self._cached_bottle_arm is None:
                    return _err('target_not_found',
                                'bottle detection/TF failed during hover',
                                None)
                return _err('ik_failed',
                            'IK could not find a reachable + collision-free '
                            'solution at stage 1 (hover)',
                            'let nav reposition for a better angle')

            # Stage 2: close hand
            self.get_logger().info('STAGE 2/3: close fingers around bottle')
            ok = self.close_hand(preset='grasp')
            time.sleep(CLOSE_SETTLE_SEC)
            if self._cancel_flag:
                return {'status': 'cancelled', 'metadata': {'stage': 'after_close'}}
            if not ok:
                return _err('hardware_fault',
                            'close_hand service did not confirm success',
                            'check left_hand bridge alive')

            # TODO(6.3): subscribe /left_hand/state, read effort post-close,
            # return error_code=grasp_empty if effort < 40 across Index+Little.
            # For MVP we trust the close + lift sequence.

            # Stage 3: lift
            self.get_logger().info('STAGE 3/3: lift bottle')
            ok = self.hover(mode='lift', fresh=False)
            if self._cancel_flag:
                return {'status': 'cancelled', 'metadata': {'stage': 'after_lift'}}
            if not ok:
                return _err('ik_failed',
                            'IK/MoveIt failed at stage 3 (lift)',
                            'lift trajectory blocked -- check scene / try again')

            # Success: update holding state
            self._holding[which] = {
                'holding_object': True,
                'object_class': object_class,
                'pose': 'lift',
            }
            return {
                'status': 'done',
                'metadata': {
                    'which_used': which,
                    'grasp_effort': 0,  # TODO: real effort from /left_hand/state
                    'holding_object': True,
                    'object_class': object_class,
                    'final_pose': 'lift',
                }
            }
        finally:
            self._busy_lock.release()

    # ---- Tool: get_arm_status ---------------------------------------------
    def _tool_get_arm_status(self):
        return {
            'status': 'done',
            'metadata': {
                'left': dict(self._holding['left']),
                'right': dict(self._holding['right']),
            }
        }

    # ---- Tool: arm_home ---------------------------------------------------
    def _tool_arm_home(self, args):
        if not self._busy_lock.acquire(blocking=False):
            return _err('hardware_fault',
                        'arm busy with another action',
                        'wait or call cancel first')
        try:
            which = args.get('which', 'auto')
            if which == 'auto':
                which = self.side
            if which != self.side:
                return _err('hardware_fault',
                            f'this client only drives side={self.side}',
                            None)

            home = LEFT_HOME_RAD if which == 'left' else RIGHT_HOME_RAD
            ok = self.move_to_joints(home, label='home')
            if not ok:
                return _err('motion_failed',
                            'home move failed (MoveIt/SDK)',
                            None)

            self._holding[which] = {
                'holding_object': False,
                'object_class': None,
                'pose': 'home',
            }
            return {
                'status': 'done',
                'metadata': {
                    'which_used': which,
                    'pose': 'home',
                    'holding_object': False,
                }
            }
        finally:
            self._busy_lock.release()

    # ---- Tool: handover_water ---------------------------------------------
    def _tool_handover_water(self, args):
        """Move the configured arm to the handover pose.

        AI team routes user utterance \"把水递给我\" to this tool.
        Currently only the left arm has a calibrated handover pose;
        requesting right (or auto-on-right deployment) returns hardware_fault.
        """
        if not self._busy_lock.acquire(blocking=False):
            return _err('hardware_fault',
                        'arm busy with another action',
                        'wait or call cancel first')
        try:
            which = args.get('which', 'auto')
            if which == 'auto':
                which = self.side
            if which != self.side:
                return _err('hardware_fault',
                            f'this client only drives side={self.side}',
                            None)
            if which != 'left':
                return _err('hardware_fault',
                            'handover_water currently only configured for left arm',
                            'use which="left" or wait for right-arm calibration')

            ok = self.move_to_joints(LEFT_HANDOVER_RAD, label='handover')
            if not ok:
                return _err('motion_failed',
                            'handover move failed (MoveIt/SDK)',
                            None)

            # Holding state unchanged -- if AI team called grab_bottle first,
            # the bottle is still in hand at this pose; if not, hand is empty.
            self._holding[which]['pose'] = 'handover'
            return {
                'status': 'done',
                'metadata': {
                    'which_used': which,
                    'pose': 'handover',
                    'holding_object': self._holding[which]['holding_object'],
                }
            }
        finally:
            self._busy_lock.release()


def main():
    import sys, os
    # WebSocket-client honors http_proxy/https_proxy/all_proxy env vars
    # and will try to tunnel to AI server via Clash/Verge on 127.0.0.1:7897.
    # Strip them so the WS lib connects directly to AI server on LAN.
    for k in ('http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY',
              'all_proxy','ALL_PROXY','no_proxy','NO_PROXY'):
        os.environ.pop(k, None)

    ai_host = DEFAULT_AI_HOST
    ai_port = DEFAULT_AI_PORT
    side = DEFAULT_SIDE
    for a in sys.argv[1:]:
        if a.startswith('--host='):
            ai_host = a.split('=', 1)[1]
        elif a.startswith('--port='):
            ai_port = int(a.split('=', 1)[1])
        elif a in ('left', 'right'):
            side = a

    rclpy.init()
    node = ArmGraspWSClient(side=side, ai_host=ai_host, ai_port=ai_port)
    node.wait_ready(30.0)

    # Start WS connect loop in background
    ws_thread = threading.Thread(target=node.run_ws_client, daemon=True)
    ws_thread.start()
    node.get_logger().info('WS thread started; spinning ROS executor')

    # MultiThreadedExecutor lets request-handler worker threads call
    # spin_until_future_complete + service calls without deadlocking the
    # main spin.
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_flag = True
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
