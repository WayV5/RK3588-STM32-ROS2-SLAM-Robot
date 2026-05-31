#!/bin/bash -e

[ "$RK_ROOTFS_FRECON" ] || exit 0

TARGET_DIR="$1"
[ "$TARGET_DIR" ] || exit 1

OVERLAY_DIR="$(dirname "$(realpath "$0")")"

echo "Installing frecon to $TARGET_DIR..."
$RK_RSYNC "$OVERLAY_DIR/usr" "$OVERLAY_DIR/etc" "$TARGET_DIR/"

install_sysv_service "$OVERLAY_DIR/S35frecon" 5 4 3 2 K02 0 1 6
install_busybox_service "$OVERLAY_DIR/S35frecon"
install_systemd_service "$OVERLAY_DIR/frecon.service" \
	"$OVERLAY_DIR/S35frecon" "/etc/init.d/frecon"
