#!/usr/bin/env python3
"""cmd_vel_mux — minimal priority Twist mux (no twist_mux dependency).

David's AGX does not ship twist_mux, and the rest of this package favours
self-contained nodes (cmd_vel_separator), so this is the default arbiter
between the position loop and the Nav2 chain. A twist_mux yaml that does the
same thing is provided in config/twist_mux_position.yaml for sites that
prefer the upstream package.

Behaviour (SPEC §6):
  * Each input has a priority; the mux forwards the highest-priority input
    that has published within ``input_timeout`` seconds. Single output ⇒ the
    two sources can never fight over the chassis.
  * On a source SWITCH it injects one zero Twist (zero-speed transition) so
    the chassis never sees a velocity step when handing off.
  * When no input is active it publishes zeros for a short grace period then
    goes silent, yielding /cmd_vel to any ad-hoc publisher (teleop).
  * ``~/enable`` (std_srvs/SetBool): data=false locks OUT the top-priority
    input (forces the Nav2 source); data=true (default) restores preemption.

Default wiring:
  /cmd_vel_position (prio 100) | /cmd_vel_nav (prio 10)  ->  /cmd_vel
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class _Input:
    __slots__ = ("topic", "priority", "last_twist", "last_stamp", "sub")

    def __init__(self, topic, priority):
        self.topic = topic
        self.priority = priority
        self.last_twist = None
        self.last_stamp = None
        self.sub = None


class CmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")

        self.declare_parameter("input_topics",
                               ["/cmd_vel_position", "/cmd_vel_nav"])
        self.declare_parameter("input_priorities", [100, 10])
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("input_timeout", 0.3)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("idle_grace_sec", 0.5)

        topics = list(self.get_parameter("input_topics").value)
        prios = list(self.get_parameter("input_priorities").value)
        if len(prios) != len(topics):
            prios = list(range(len(topics), 0, -1))
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.rate = max(1.0, float(self.get_parameter("rate").value))
        self.idle_grace_sec = float(self.get_parameter("idle_grace_sec").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # build inputs, sorted by priority desc (index 0 = top priority)
        self.inputs = [_Input(t, int(p)) for t, p in zip(topics, prios)]
        self.inputs.sort(key=lambda i: i.priority, reverse=True)
        for inp in self.inputs:
            inp.sub = self.create_subscription(
                Twist, inp.topic, self._make_cb(inp), qos)

        self.pub = self.create_publisher(Twist, self.output_topic, qos)
        self.enabled = True   # top-priority input preemption allowed
        self.srv = self.create_service(SetBool, "~/enable", self._on_enable)

        self._active_topic = None       # which input is currently forwarded
        self._idle_since = None
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self.get_logger().info(
            "cmd_vel_mux up: "
            + " | ".join(f"{i.topic}(p{i.priority})" for i in self.inputs)
            + f"  ->  {self.output_topic}")

    def _make_cb(self, inp: _Input):
        def _cb(msg: Twist) -> None:
            inp.last_twist = msg
            inp.last_stamp = self.get_clock().now()
        return _cb

    def _on_enable(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = ("position-loop preemption ON" if self.enabled
                            else "position-loop preemption OFF (Nav2 forced)")
        self.get_logger().info(response.message)
        return response

    def _is_active(self, inp: _Input, now) -> bool:
        if inp.last_stamp is None:
            return False
        return (now - inp.last_stamp).nanoseconds * 1e-9 <= self.input_timeout

    def _tick(self) -> None:
        now = self.get_clock().now()
        chosen = None
        for idx, inp in enumerate(self.inputs):
            # top-priority input is locked out when disabled
            if idx == 0 and not self.enabled:
                continue
            if self._is_active(inp, now):
                chosen = inp
                break

        if chosen is None:
            # nobody driving — emit zeros for a grace window then go silent
            if self._active_topic is not None:
                self._idle_since = now
                self._active_topic = None
            if self._idle_since is not None:
                idle_age = (now - self._idle_since).nanoseconds * 1e-9
                if idle_age <= self.idle_grace_sec:
                    self.pub.publish(Twist())   # zero
            return

        # zero-speed transition on any source switch
        if self._active_topic is not None and self._active_topic != chosen.topic:
            self.pub.publish(Twist())
            self.get_logger().info(
                f"mux switch {self._active_topic} -> {chosen.topic} (zero gap)")
        self._active_topic = chosen.topic
        self._idle_since = None
        self.pub.publish(chosen.last_twist)


def main(argv=None):
    rclpy.init(args=argv)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
