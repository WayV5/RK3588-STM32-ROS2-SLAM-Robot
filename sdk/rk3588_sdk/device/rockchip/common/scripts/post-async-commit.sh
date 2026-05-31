#!/bin/bash -e

POST_ROOTFS_ONLY=1

source "${POST_HELPER:-$(dirname "$(realpath "$0")")/../post-hooks/post-helper}"

echo "Installing async-commit service..."

rm -f etc/init.d/S*_commit.sh \
	etc/systemd/system/multi-user.target.wants/async.service \
	usr/lib/systemd/system/async.service

cd "$SDK_DIR"

mkdir -p "$TARGET_DIR/usr/bin"
install -m 0755 external/rkscript/async-commit "$TARGET_DIR/usr/bin/"

install -m 0755 "$RK_TOOL_DIR/armhf/modetest" "$TARGET_DIR/usr/bin/"

install_sysv_service external/rkscript/S*async-commit.sh S
install_busybox_service external/rkscript/S*async-commit.sh
install_systemd_service external/rkscript/async-commit.service
