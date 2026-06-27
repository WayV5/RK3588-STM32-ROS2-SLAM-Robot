#!/usr/bin/env python3
"""
patrol.py — multi-waypoint cruise for Nav2 autonomous navigation demo.

Each corner is split into arrive+rotate: robot arrives straight, rotates on the spot,
then goes straight to the next corner — no arc, no wall collision.

Usage:
    python3 patrol.py              # one circuit (transit + 1 loop)
    python3 patrol.py --loop       # transit once, then loop forever
    python3 patrol.py --timeout 30 # custom goal timeout (s)
"""

import math
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


# Smoothed rounded-rectangle polygon — straight edges + quarter-circle arcs at corners.
# Each waypoint yaw = direction to the next waypoint.  Max turn ≤20°.
def _build_waypoints():
    import math as _m
    xl, xr = 1.35, 1.70
    yb, yt = -0.10, 1.25
    r = 0.08                      # corner radius (m)
    sp = 0.12                     # straight-edge point spacing (m)

    def _edge(x1, y1, x2, y2):
        dist = _m.hypot(x2 - x1, y2 - y1)
        n = max(2, round(dist / sp))
        return [((x1 + (x2 - x1) * i / (n - 1)), (y1 + (y2 - y1) * i / (n - 1)))
                for i in range(n)]

    def _arc(cx, cy, a1, a2, n):
        return [((cx + r * _m.cos(a1 + (a2 - a1) * i / (n - 1))),
                 (cy + r * _m.sin(a1 + (a2 - a1) * i / (n - 1))))
                for i in range(n)]

    ha = _m.pi / 2  # 90°
    # Arc tangent = edge direction → arc angle = edge_direction - 90°
    # Bottom→Right: tangent 0°→90°, arc -90°→0°  |  Right→Top: 90°→180°, arc 0°→90°
    # Top→Left:     180°→270°, arc 90°→180°     |  Left→Bottom: 270°→360°, arc 180°→270°
    raw = []
    raw += _edge(xl + r, yb, xr - r, yb)[:-1]                          # bottom edge →
    raw += _arc(xr - r, yb + r, -ha, 0, 4)[:-1]                        # BR corner  ↗
    raw += _edge(xr, yb + r, xr, yt - r)[:-1]                          # right edge  ↑
    raw += _arc(xr - r, yt - r, 0, ha, 4)[:-1]                         # TR corner  ↖
    raw += _edge(xr - r, yt, xl + r, yt)[:-1]                          # top edge    ←
    raw += _arc(xl + r, yt - r, ha, 2 * ha, 4)[:-1]                    # TL corner  ↙
    raw += _edge(xl, yt - r, xl, yb + r)[:-1]                          # left edge   ↓
    raw += _arc(xl + r, yb + r, 2 * ha, 3 * ha, 4)                     # BL corner  ↘

    wps = []
    for i, (x, y) in enumerate(raw):
        nx, ny = raw[(i + 1) % len(raw)]
        yaw = _m.atan2(ny - y, nx - x)
        qz = _m.sin(yaw / 2)
        qw = _m.cos(yaw / 2)
        wps.append((round(x, 3), round(y, 3), 0.0, round(qz, 4), round(qw, 4)))
    return wps

WAYPOINTS = _build_waypoints()
# ~30 points, max turn ≤20°


class PatrolNode(Node):
    def __init__(self, waypoints, timeout, loop_forever):
        super().__init__("patrol")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._waypoints = waypoints
        self._timeout = timeout
        self._loop_forever = loop_forever
        self._goal_idx = 0
        self._goal_pending = False
        self._next_timer = None
        self._result_done = False

        self.get_logger().info(f"{len(waypoints)} waypoints, loop={loop_forever}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available")
            raise RuntimeError("Action server not available")
        self.get_logger().info("Connected — starting patrol")

    def _cancel_next_timer(self):
        if self._next_timer is not None:
            self._next_timer.cancel()
            self._next_timer = None

    def _schedule_next(self, delay=0.2):
        self._cancel_next_timer()
        self._next_timer = self.create_timer(delay, self._send_next_from_timer)

    def _send_next_from_timer(self):
        self._next_timer = None
        self.send_next()

    def send_next(self):
        if self._goal_pending:
            self.get_logger().warning("send_next skipped — goal still pending")
            return

        if self._goal_idx >= len(self._waypoints):
            if self._loop_forever:
                self._goal_idx = 0
                self.get_logger().info("=== Loop restart ===")
            else:
                self.get_logger().info("=== Patrol complete ===")
                rclpy.shutdown()
                return

        x, y, _, qz, qw = self._waypoints[self._goal_idx]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        heading = 2 * math.degrees(math.asin(min(abs(qz), 1.0)))
        self.get_logger().info(f"→ WP{self._goal_idx + 1}: ({x:.2f}, {y:.2f}) heading ~{heading:.0f}°")
        self._result_done = False
        self._goal_pending = True
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"WP{self._goal_idx + 1} goal REJECTED — skipping")
            self._goal_pending = False
            self._goal_idx += 1
            self._schedule_next(1.0)
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        if self._result_done:
            self.get_logger().warning(f"WP{self._goal_idx + 1} _on_result double-fire — ignored")
            return
        self._result_done = True
        self._goal_pending = False
        status = future.result().status
        idx = self._goal_idx + 1
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"WP{idx} ✓")
            delay = 0.2
        else:
            self.get_logger().warning(f"WP{idx} status={status}")
            delay = 2.0
        self._goal_idx += 1
        self._schedule_next(delay)


def main():
    parser = argparse.ArgumentParser(description="Patrol cruise with Nav2")
    parser.add_argument("--loop", action="store_true", help="Loop waypoints indefinitely")
    parser.add_argument("--timeout", type=float, default=60.0, help="Goal timeout (s)")
    args = parser.parse_args()

    time.sleep(8)  # let Nav2 stabilize after launch
    rclpy.init()
    try:
        node = PatrolNode(WAYPOINTS, args.timeout, args.loop)
        node.send_next()
        rclpy.spin(node)
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
