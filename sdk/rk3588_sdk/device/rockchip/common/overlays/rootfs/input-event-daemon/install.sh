#!/bin/bash -e

[ -z "$RK_ROOTFS_INPUT_EVENT_DAEMON_DISABLED" ] || exit 0
if [ "$RK_ROOTFS_INPUT_EVENT_DAEMON_DEFAULT" ]; then
	[ "$POST_OS" = yocto ] || exit 0
	echo -e "\e[33mInstall input-event-daemon for yocto by default\e[0m"
fi

TARGET_DIR="$1"
[ "$TARGET_DIR" ] || exit 1

OVERLAY_DIR="$(dirname "$(realpath "$0")")"

echo "Installing input-event-daemon to $TARGET_DIR..."

mkdir -p "$TARGET_DIR/etc" "$TARGET_DIR/lib" "$TARGET_DIR/usr/bin"

find "$TARGET_DIR/etc" "$TARGET_DIR/lib" "$TARGET_DIR/usr/bin" \
	-name "*triggerhappy*" -print0 | xargs -0 rm -rf

$RK_RSYNC "$OVERLAY_DIR/usr" "$OVERLAY_DIR/etc" "$TARGET_DIR/"

install_sysv_service "$OVERLAY_DIR/S99input-event-daemon" 5 4 3 2 K01 0 1 6
install_busybox_service "$OVERLAY_DIR/S99input-event-daemon"
install_systemd_service "$OVERLAY_DIR/input-event-daemon.service"
