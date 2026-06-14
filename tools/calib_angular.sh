#!/bin/bash
# calib_angular.sh — 标定角速度方差 (twist.covariance[35])
#
# Usage:
#   ./calib_angular.sh [speed]    默认 0.5 rad/s
#
# 发恒定角速度 → 记录 /odom twist.angular.z → Ctrl+C 打印方差
#
set -euo pipefail

SPEED="${1:-0.5}"
TOPIC="/odom"
FIELD="twist.twist.angular.z"

echo "===== Angular covariance calibration ====="
echo "  speed: ${SPEED} rad/s"
echo "  Press Ctrl+C to stop and print result"
echo ""

# Start constant command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
	"{linear: {x: 0.0}, angular: {z: ${SPEED}}}" --rate 50 &
PUB_PID=$!
sleep 1

# Record + compute running variance
stdbuf -oL ros2 topic echo "$TOPIC" --field "$FIELD" 2>/dev/null | \
awk -v speed="$SPEED" '
{
	sum   += $1
	sumsq += $1 * $1
	count++
	mean   = sum / count
	variance = (sumsq / count) - (mean * mean)
	stddev   = sqrt(variance < 0 ? 0 : variance)
	printf "\r  n=%d  mean=%.4f  variance=%.8f  stddev=%.6f rad/s", count, mean, variance, stddev
}
END {
	printf "\n"
	print "===== Result ====="
	printf "  twist.covariance[35] = %.8f\n", variance
	printf "  stddev               = %.6f rad/s\n", stddev
}
'
kill $PUB_PID 2>/dev/null || true
