#!/bin/bash -e

POST_ROOTFS_ONLY=1

source "${POST_HELPER:-$(dirname "$(realpath "$0")")/../post-hooks/post-helper}"

[ -z "$RK_DISK_HELPERS_DISABLED" ] || exit 0

cd "$SDK_DIR"

mkdir -p "$TARGET_DIR/usr/bin"
install -m 0755 external/rkscript/disk-helper "$TARGET_DIR/usr/bin/"

if [ "$RK_DISK_HELPERS_MOUNTALL" ]; then
	DISK_HELPER_TYPE=mount
elif [ "$RK_DISK_HELPERS_RESIZEALL" ]; then
	DISK_HELPER_TYPE=resize
else
	if [ "$POST_OS" = buildroot ]; then
		DISK_HELPER_TYPE=mount
	else
		DISK_HELPER_TYPE=resize
	fi
fi

echo "Installing $DISK_HELPER_TYPE service..."

install -m 0755 external/rkscript/$DISK_HELPER_TYPE-helper \
	"$TARGET_DIR/usr/bin/"

SCRIPT=$(ls external/rkscript/ | grep ${DISK_HELPER_TYPE}all.sh)

install_busybox_service external/rkscript/$SCRIPT

if [ "$DISK_HELPER_TYPE" = mount ]; then
	if [ "$RK_DISK_AUTO_FORMAT" ]; then
		echo "Enabling auto formatting..."
		touch "$TARGET_DIR/.auto_mkfs"
	fi

	if [ "$RK_DISK_SKIP_FSCK" ]; then
		echo "Disabling boot time fsck..."
		touch "$TARGET_DIR/.skip_fsck"
	fi
	exit 0
fi

install_sysv_service external/rkscript/$SCRIPT S
install_systemd_service external/rkscript/$DISK_HELPER_TYPE-all.service
