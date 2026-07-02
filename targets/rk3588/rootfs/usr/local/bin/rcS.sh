#!/bin/bash
# rcS.sh — boot-time startup script for RK3588 robot
# Triggered by rcS.service (systemd oneshot) at multi-user.target

set -e

FLAG_DIR="/var/lib/rcS"
mkdir -p "$FLAG_DIR"

# ─── 1. Resize rootfs + app partition on first boot ─────────────────
if [ ! -f "$FLAG_DIR/resize-done" ]; then
    echo "rcS: resizing rootfs to fill partition..."
    resize2fs /dev/mmcblk0p6
    resize2fs /dev/mmcblk0p8
    touch "$FLAG_DIR/resize-done"
    echo "rcS: resize done"
fi

# ─── 2. CPU/NPU performance governor ───────────────────────────────
/usr/local/bin/set_performance.sh || echo "rcS: set_performance failed"

# ─── 2.5. IRQ affinity — bind non-RT interrupts to little cores ──────
/usr/local/bin/set_irq_affinity.sh || echo "rcS: set_irq_affinity failed"

# ─── 3. wifi — start wpa_supplicant in background ───────────────────
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf

echo "rcS: done"
