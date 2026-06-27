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


# Transit: origin → WP1 north (once only, heading 0° = same as robot start)
TRANSIT = (1.3, -0.02, 0.0, 0.0, 1.0)  # heading 0° north

# Loop: WP2→WP3→WP4→WP1→WP2, each corner = arrive + rotate
#   arrive: face travel direction (straight line to next corner)
#   rotate: same position, 90° left turn on the spot
#   WP2 first: TRANSIT→WP2 同向(0°北), 无旋转, 无缝衔接
LOOP_WAYPOINTS = [
    # WP2: arrive from WP1 heading north, rotate to west
    (1.88, -0.02, 0.0,  0.0000, 1.0000),  # arrive face north (0°)
    (1.88, -0.02, 0.0,  0.7071, 0.7071),  # rotate face west (90°)
    # WP3: arrive from WP2 heading west, rotate to south
    (1.88,  1.25, 0.0,  0.7071, 0.7071),  # arrive face west (90°)
    (1.88,  1.25, 0.0,  1.0000, 0.0000),  # rotate face south (180°)
    # WP4: arrive from WP3 heading south, rotate to east
    (1.3,   1.25, 0.0,  1.0000, 0.0000),  # arrive face south (180°)
    (1.3,   1.25, 0.0, -0.7071, 0.7071),  # rotate face east (-90°)
    # WP1: arrive from WP4 heading east, rotate to north
    (1.3,  -0.02, 0.0, -0.7071, 0.7071),  # arrive face east (-90°)
    (1.3,  -0.02, 0.0,  0.0000, 1.0000),  # rotate face north (0°)
]


class PatrolNode(Node):
    def __init__(self, transit, loop_wps, timeout, loop_forever):
        super().__init__("patrol")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._transit = transit
        self._loop_wps = loop_wps
        self._timeout = timeout
        self._loop_forever = loop_forever
        self._goal_idx = 0
        self._in_transit = True
        self._goal_pending = False
        self._next_timer = None
        self._result_done = False

        self.get_logger().info(f"Transit + {len(loop_wps)} loop WPs, loop={loop_forever}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available")
            raise RuntimeError("Action server not available")
        self.get_logger().info("Connected — starting patrol")

    def _cancel_next_timer(self):
        if self._next_timer is not None:
            self._next_timer.cancel()
            self._next_timer = None

    def _schedule_next(self, delay=1.0):
        self._cancel_next_timer()
        self._next_timer = self.create_timer(delay, self._send_next_from_timer)

    def _send_next_from_timer(self):
        self._next_timer = None
        self.send_next()

    def _build_goal(self, wp):
        x, y, _, qz, qw = wp
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def send_next(self):
        if self._goal_pending:
            self.get_logger().warning("send_next skipped — goal still pending")
            return

        if self._in_transit:
            self._in_transit = False
            goal = self._build_goal(self._transit)
            self.get_logger().info(f"→ TRANSIT: ({self._transit[0]:.2f}, {self._transit[1]:.2f}) heading 0°")
        else:
            if self._goal_idx >= len(self._loop_wps):
                if self._loop_forever:
                    self._goal_idx = 0
                    self.get_logger().info("=== Loop restart ===")
                else:
                    self.get_logger().info("=== Patrol complete ===")
                    rclpy.shutdown()
                    return
            wp = self._loop_wps[self._goal_idx]
            goal = self._build_goal(wp)
            heading = 2 * math.degrees(math.asin(min(abs(wp[3]), 1.0)))
            label = "arrive" if self._goal_idx % 2 == 0 else "rotate"
            corner = self._goal_idx // 2 + 1
            self.get_logger().info(f"→ WP{corner} {label}: ({wp[0]:.2f}, {wp[1]:.2f}) heading ~{heading:.0f}°")

        self._result_done = False
        self._goal_pending = True
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("goal REJECTED — skipping")
            self._goal_pending = False
            if not self._in_transit:
                self._goal_idx += 1
            self._schedule_next(1.0)
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        if self._result_done:
            self.get_logger().warning("_on_result double-fire — ignored")
            return
        self._result_done = True
        self._goal_pending = False
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info("✓")
            delay = 0.5  # short pause, rotate is instant
        else:
            self.get_logger().warning(f"status={status}")
            delay = 3.0
        if not self._in_transit:
            self._goal_idx += 1
        self._schedule_next(delay)


def main():
    parser = argparse.ArgumentParser(description="Patrol cruise with Nav2")
    parser.add_argument("--loop", action="store_true", help="Loop waypoints indefinitely")
    parser.add_argument("--timeout", type=float, default=60.0, help="Goal timeout (s)")
    args = parser.parse_args()

    time.sleep(5)  # let Nav2 stabilize after launch
    rclpy.init()
    try:
        node = PatrolNode(TRANSIT, LOOP_WAYPOINTS, args.timeout, args.loop)
        node.send_next()
        rclpy.spin(node)
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
