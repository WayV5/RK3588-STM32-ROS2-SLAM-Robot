#!/bin/bash
# calib_linear.sh — 标定直线速度方差 (twist.covariance[0])
#
# Usage:
#   ./calib_linear.sh [speed]     默认 0.3 m/s
#
# 发恒定直线速度 → 记录 /odom twist.linear.x → Ctrl+C 打印方差
#
set -euo pipefail

SPEED="${1:-0.3}"
TOPIC="/odom"
FIELD="twist.twist.linear.x"

echo "===== Linear covariance calibration ====="
echo "  speed: ${SPEED} m/s"
echo "  Press Ctrl+C to stop and print result"
echo ""

# Start constant command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
	"{linear: {x: ${SPEED}}, angular: {z: 0.0}}" --rate 50 &
PUB_PID=$!
sleep 1  # let robot reach steady state

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
	printf "\r  n=%d  mean=%.4f  variance=%.8f  stddev=%.6f m/s", count, mean, variance, stddev
}
END {
	printf "\n"
	print "===== Result ====="
	printf "  twist.covariance[0] = %.8f\n", variance
	printf "  stddev              = %.6f m/s\n", stddev
}
'
kill $PUB_PID 2>/dev/null || true
