#!/bin/bash
# RK3588 性能模式 — CPU/NPU 定频到最高，消除延迟抖动
# Usage: sudo bash tools/set_performance.sh
set -euo pipefail

echo "=== RK3588 Performance Mode ==="

# ── CPU: all cores → performance governor ──────────────────────
echo "[CPU] Setting all cores to performance governor..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$cpu" 2>/dev/null || true
done

echo "  Governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
echo "  A55 freq: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) Hz"
echo "  A76 freq: $(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq) Hz"

# ── NPU: lock to 1GHz (max) ────────────────────────────────────
NPU_DEVFREQ=/sys/class/devfreq/fdab0000.npu
if [ -d "$NPU_DEVFREQ" ]; then
    echo "[NPU] Setting governor to performance..."
    echo performance > "$NPU_DEVFREQ/governor" 2>/dev/null || true
    echo "  Governor: $(cat $NPU_DEVFREQ/governor)"
    echo "  Frequency: $(cat $NPU_DEVFREQ/cur_freq) Hz"
else
    echo "[NPU] devfreq not found, trying debugfs..."
    echo 1000000000 > /sys/kernel/debug/rknpu/freq 2>/dev/null || echo "  NPU freq fix skipped"
fi

echo ""
echo "=== Done ==="
echo "A76 @ 2.26GHz | A55 @ 1.8GHz | NPU @ 1GHz"
echo ""
echo "Verify: ros2 topic hz /detections  (should be stable)"
