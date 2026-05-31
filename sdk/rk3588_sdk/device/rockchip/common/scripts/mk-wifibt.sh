#!/bin/bash -e

build_wifibt()
{
	DEFAULT_ROOTFS_DIR=buildroot/output/$RK_BUILDROOT_CFG/target
	ROOTFS_DIR="${1:-$DEFAULT_ROOTFS_DIR}"
	if [ ! -d "$ROOTFS_DIR" ]; then
		echo "$ROOTFS_DIR is not a dir!"
		return 1
	fi

	RK_WIFIBT_CHIP="${2:-$RK_WIFIBT_CHIP}"
	"$SCRIPTS_DIR/post-wifibt.sh" "$(realpath "$ROOTFS_DIR")" \
		$([ -r "$ROOTFS_DIR/etc/os-release" ] || echo buildroot)
	finish_build build_wifibt $@
}

# Hooks

usage_hook()
{
	echo -e "wifibt[:<dst dir>[:<chip>]]       \tbuild Wifi/BT"
}

BUILD_CMDS="wifibt"
build_hook()
{
	shift
	build_wifibt $@
}

source "${BUILD_HELPER:-$(dirname "$(realpath "$0")")/../build-hooks/build-helper}"

build_wifibt $@
