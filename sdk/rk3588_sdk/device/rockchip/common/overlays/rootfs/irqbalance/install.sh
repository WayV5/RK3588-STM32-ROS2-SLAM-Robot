#!/bin/bash -e

[ "$RK_ROOTFS_IRQBALANCE" ] || exit 0

TARGET_DIR="$1"
[ "$TARGET_DIR" ] || exit 1

OVERLAY_DIR="$(dirname "$(realpath "$0")")"

echo "Installing irqbalance to $TARGET_DIR..."
$RK_RSYNC "$OVERLAY_DIR/usr" "$OVERLAY_DIR/etc" "$TARGET_DIR/"

install_sysv_service "$OVERLAY_DIR/S13irqbalance" 5 4 3 2 K04 0 1 6
install_busybox_service "$OVERLAY_DIR/S13irqbalance"
install_systemd_service "$OVERLAY_DIR/irqbalance.service"
