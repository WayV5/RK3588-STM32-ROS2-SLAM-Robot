#!/usr/bin/env python3
"""
patrol.py — multi-waypoint cruise for Nav2 autonomous navigation demo.

Usage:
    python3 patrol.py                          # 4-waypoint circuit, once
    python3 patrol.py --loop                    # loop all waypoints
    python3 patrol.py --loop --loop-from 1       # WP1 once, then loop WP2-WP5
    python3 patrol.py --timeout 30             # custom goal timeout (s)
"""

import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


# 5-waypoint circuit (tested from /odom, 2026-06-27)
WAYPOINTS = [
    (1.1678, -0.0243, 0.0, -0.0211, 0.9998),   # WP1: east 1.2m
    (1.8828, -0.0791, 0.0,  0.8672, 0.4979),   # WP2: SE corner, face ~120° (+0.05 X)
    (1.3022,  0.9466, 0.0,  0.8591, 0.5118),   # WP3: north, face ~118°  (+0.05 X)
    (1.0679,  0.4396, 0.0, -0.8469, 0.5317),   # WP4: mid-field, face ~-116° (+0.05 X)
    (1.8828, -0.0791, 0.0,  0.8672, 0.4979),   # WP5: return to WP2 (+0.05 X)
]


class PatrolNode(Node):
    def __init__(self, waypoints: list, timeout: float, loop_forever: bool, loop_from: int):
        super().__init__("patrol")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._waypoints = waypoints
        self._timeout = timeout
        self._loop_forever = loop_forever
        self._loop_from = loop_from
        self._goal_idx = 0
        self._goal_pending = False  # prevent concurrent goal sends

        self.get_logger().info(f"Waypoints: {len(waypoints)}, loop={loop_forever}, loop_from={loop_from}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available")
            raise RuntimeError("Action server not available")
        self.get_logger().info("Connected to navigate_to_pose — starting patrol")

    def send_next(self):
        if self._goal_pending:
            return  # already waiting for result, skip duplicate
        if self._goal_idx >= len(self._waypoints):
            if self._loop_forever:
                self._goal_idx = self._loop_from
                self.get_logger().info(f"=== Loop restart (idx={self._loop_from}) ===")
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
        self._goal_pending = True
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"WP{self._goal_idx + 1} goal REJECTED — skipping")
            self._goal_pending = False
            self._goal_idx += 1
            self.send_next()
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._goal_pending = False
        status = future.result().status
        idx = self._goal_idx + 1
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"WP{idx} ✓")
        else:
            self.get_logger().warning(f"WP{idx} status={status}")
        self._goal_idx += 1
        self.create_timer(1.0, self.send_next)  # 1s pause between goals


def main():
    parser = argparse.ArgumentParser(description="Patrol cruise with Nav2")
    parser.add_argument("--loop", action="store_true", help="Loop waypoints indefinitely")
    parser.add_argument("--loop-from", type=int, default=0, help="When looping, restart from this index (default 0)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Goal timeout (s)")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = PatrolNode(WAYPOINTS, args.timeout, args.loop, args.loop_from)
        node.send_next()
        rclpy.spin(node)
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
