#!/usr/bin/env python3
"""HTTP server for AI orchestration to trigger grasps.

2026-06-02: Minimal MVP per AI team request:
    POST /grab    -> trigger grasp, block until done (30s timeout)
                     return {"ok": true} on success
                     return {"ok": false, "reason": "not_detected|grasp_failed|hardware_error",
                             "message": "..."} on failure
    GET  /health  -> readiness check, returns {"ok": true, "status": "ready"}

Listens on 0.0.0.0:9100 (no auth, intranet only).

Implementation: subclasses PickFromCam so all the side-aware grasp logic
(workspace pre-check, wrist constraint, IK, bottle cache, etc.) is reused
verbatim. The HTTP layer is just a thin adapter to the AI org schema.

Side selection: hardcoded to 'left' for now since the right hand thumb is
out for repair (2026-06-01). Flip DEFAULT_SIDE to 'right' once it's back.
"""
import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy

from david_grasp.pick_from_cam import PickFromCam, CLOSE_SETTLE_SEC


DEFAULT_PORT = 9100
DEFAULT_SIDE = 'left'  # right is out for thumb repair as of 2026-06-01


class GrabHandler(BaseHTTPRequestHandler):
    """HTTP handler. The single global server_node is set in main()."""
    server_node = None

    def log_message(self, fmt, *args):
        # Silence default access log -- ROS logger gives us what we need.
        pass

    def do_POST(self):
        if self.path != '/grab':
            self._send(404, {
                "ok": False,
                "reason": "hardware_error",
                "message": f"unknown path {self.path}",
            })
            return
        # Optionally drain any small JSON body the AI team might send later
        # (their note mentions future coord params); ignore for now.
        clen = int(self.headers.get('Content-Length', '0') or '0')
        if clen > 0:
            try:
                self.rfile.read(clen)
            except Exception:
                pass
        result = GrabHandler.server_node.execute_grab()
        self._send(200, result)

    def do_GET(self):
        if self.path == '/health':
            self._send(200, {"ok": True, "status": "ready"})
            return
        self._send(404, {
            "ok": False,
            "reason": "hardware_error",
            "message": f"unknown path {self.path}",
        })

    def _send(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class GrabServerNode(PickFromCam):
    def __init__(self, side=DEFAULT_SIDE):
        super().__init__(side=side)
        self._busy_lock = threading.Lock()
        self.get_logger().info(
            f'GrabServerNode init (side={side}); HTTP layer will start in main()')

    def execute_grab(self):
        """Run the full grasp pipeline and map outcomes to AI team schema.

        Reasons:
            not_detected   - no fresh PointStamped, bottle TF failed, or workspace violation
            grasp_failed   - IK no solution / MoveIt motion failed / lift failed / close hand silent fail
            hardware_error - SDK error, service unavailable, internal exception, or contention
        """
        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warn('/grab arrived while previous grasp in-flight; rejecting')
            return {
                "ok": False,
                "reason": "hardware_error",
                "message": "another grasp already in-flight",
            }
        try:
            t0 = time.time()
            self.get_logger().info('=== HTTP /grab received, starting grasp ===')

            # Fresh session: clear caches so we re-detect the bottle.
            self._cached_bottle_arm = None
            self._cached_hover_joints = None

            # Pre-check: do we have a bottle PointStamped at all?
            if self._last_point is None:
                msg = 'no PointStamped received yet from /yolo/bottle_center'
                self.get_logger().warn(f'/grab pre-check fail: {msg}')
                return {"ok": False, "reason": "not_detected", "message": msg}

            # Stage 1: open hand + move to hover/grasp pose (fresh detection,
            # caches bottle_arm + hover joints for stage 3).
            self.get_logger().info('=== STAGE 1/3: open + move to grasp pose ===')
            t1 = time.time()
            ok = self.hover(mode='hover', fresh=True)
            self.get_logger().info(f'    stage 1 took {time.time()-t1:.2f}s')
            if not ok:
                # Distinguish "couldn't find/transform bottle" vs "IK/motion failed".
                # _get_bottle_arm sets _cached_bottle_arm on success; if still
                # None here it failed at detection/TF/workspace stage.
                if self._cached_bottle_arm is None:
                    msg = ('bottle detection / TF / workspace pre-check failed '
                           '(see ROS log for exact reason)')
                    self.get_logger().warn(f'/grab stage 1 detection-side fail: {msg}')
                    return {"ok": False, "reason": "not_detected", "message": msg}
                msg = 'IK or MoveIt motion failed at stage 1 (hover)'
                self.get_logger().warn(f'/grab stage 1 motion-side fail: {msg}')
                return {"ok": False, "reason": "grasp_failed", "message": msg}

            # Stage 2: close hand
            self.get_logger().info('=== STAGE 2/3: close hand ===')
            t2 = time.time()
            ok = self.close_hand(preset='grasp')
            time.sleep(CLOSE_SETTLE_SEC)
            self.get_logger().info(f'    stage 2 took {time.time()-t2:.2f}s')
            if not ok:
                msg = 'close_hand service did not confirm success'
                self.get_logger().warn(f'/grab stage 2 fail: {msg}')
                return {"ok": False, "reason": "hardware_error", "message": msg}

            # TODO(6.3): subscribe /{side}_hand/state, check Index/Little Pitch
            # effort >= 40 after close. If < threshold -> grasp_empty (mapped
            # to grasp_failed per AI team schema).

            # Stage 3: lift, using cached bottle position to avoid drift.
            self.get_logger().info('=== STAGE 3/3: lift (cached bottle, warm IK) ===')
            t3 = time.time()
            ok = self.hover(mode='lift', fresh=False)
            self.get_logger().info(f'    stage 3 took {time.time()-t3:.2f}s')
            if not ok:
                msg = 'IK or MoveIt motion failed at stage 3 (lift)'
                self.get_logger().warn(f'/grab stage 3 fail: {msg}')
                return {"ok": False, "reason": "grasp_failed", "message": msg}

            dt = time.time() - t0
            self.get_logger().info(f'=== /grab SUCCESS in {dt:.2f}s ===')
            return {"ok": True}

        except Exception as e:
            self.get_logger().error(f'/grab uncaught exception: {type(e).__name__}: {e}')
            return {
                "ok": False,
                "reason": "hardware_error",
                "message": f'{type(e).__name__}: {e}',
            }
        finally:
            self._busy_lock.release()


def main():
    rclpy.init()
    node = GrabServerNode(side=DEFAULT_SIDE)
    node.wait_ready(30.0)

    GrabHandler.server_node = node
    httpd = HTTPServer(('0.0.0.0', DEFAULT_PORT), GrabHandler)

    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f'HTTP server listening on 0.0.0.0:{DEFAULT_PORT}')
    node.get_logger().info(f'  POST /grab   -> trigger grasp ({DEFAULT_SIDE} arm, 30s blocking)')
    node.get_logger().info(f'  GET  /health -> readiness check')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
