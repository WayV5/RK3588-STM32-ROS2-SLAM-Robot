#!/bin/bash -e

TARGET_DIR="$1"
[ "$TARGET_DIR" ] || exit 1

OVERLAY_DIR="$(dirname "$(realpath "$0")")"

# Login root on serial console
if [ -r "$TARGET_DIR/etc/inittab" ]; then
	sed -i 's~\(respawn:\)/bin/start_getty.*~\1/bin/login -p root~' \
		"$TARGET_DIR/etc/inittab"
fi

# Use uid to detect root user
if [ -r "$TARGET_DIR/etc/profile" ]; then
	sed -i 's~"$HOME" != "/home/root"~$(id -u) -ne 0~' \
		"$TARGET_DIR/etc/profile"
fi

# Install weston overlays
if [ -x "$TARGET_DIR/usr/bin/weston" ]; then
	sed -i 's/\(WESTON_USER=\)weston/\1root/' \
		"$TARGET_DIR/etc/init.d/weston"

	echo "Installing weston overlay: $OVERLAY_DIR/weston to $TARGET_DIR..."
	$RK_RSYNC "$OVERLAY_DIR/weston/" "$TARGET_DIR/" \
		--exclude="$(basename "$(realpath "$0")")"

	echo "Installing Rockchip test scripts to $TARGET_DIR..."
	$RK_RSYNC "$SDK_DIR/external/rockchip-test/" \
		"$TARGET_DIR/rockchip-test/" \
		--include="camera/" --include="video/" --exclude="/*"
fi

# Install usbmount
if [ "$RK_YOCTO_USBMOUNT" ]; then
	mkdir -p "$TARGET_DIR/usr/bin/"
	install -m 0755 "$RK_TOOL_DIR/armhf/lockfile-create" \
		"$TARGET_DIR/usr/bin/"
	install -m 0755 "$RK_TOOL_DIR/armhf/lockfile-remove" \
		"$TARGET_DIR/usr/bin/"

	tar xvf "$OVERLAY_DIR/usbmount.tar" -C "$TARGET_DIR"

	for type in storage udisk sdcard; do
		mkdir -p "$TARGET_DIR/media/$type"{1,2,3}
		mkdir -p "$TARGET_DIR/mnt/$type"
		rm -rf "$TARGET_DIR/media/${type}0"
		ln -sf "/mnt/$type" "$TARGET_DIR/media/${type}0"
	done
fi
