#!/bin/bash -e

build_buildroot()
{
	check_config RK_BUILDROOT_CFG || return 0

	ROOTFS_DIR="${1:-$RK_OUTDIR/buildroot}"

	BUILDROOT_VERSION=$(grep "export BR2_VERSION := " \
		"$SDK_DIR/buildroot/Makefile" | xargs -n 1 | tail -n 1)

	echo "=========================================="
	echo "          Start building buildroot($BUILDROOT_VERSION)"
	echo "=========================================="

	/usr/bin/time -f "you take %E to build buildroot" \
		"$SCRIPTS_DIR/mk-buildroot.sh" $RK_BUILDROOT_CFG "$ROOTFS_DIR"

	cat "$RK_LOG_DIR/post-rootfs.log"
	finish_build build_buildroot $@
}

build_yocto()
{
	check_config RK_YOCTO_CFG RK_KERNEL_VERSION_REAL || return 0

	ROOTFS_DIR="${1:-$RK_OUTDIR/yocto}"

	"$SCRIPTS_DIR/check-yocto.sh"

	cd yocto
	rm -f build/conf/local.conf

	if echo "$RK_YOCTO_CFG" | grep -q ".conf$"; then
		if [ ! -r "$RK_YOCTO_CFG" ]; then
			RK_YOCTO_CFG="build/conf/$RK_YOCTO_CFG"
		fi

		ln -rsf "$RK_YOCTO_CFG" build/conf/local.conf

		echo "=========================================="
		echo "          Start building $RK_YOCTO_CFG"
		echo "=========================================="
	else
		{
			echo "include include/common.conf"
			echo "include include/debug.conf"
			echo "include include/display.conf"
			echo "include include/multimedia.conf"
			echo "include include/audio.conf"

			echo
			echo "MACHINE = \"$RK_YOCTO_CFG\""
			if [ "$RK_CHIP" = "rk3288w" ]; then
				echo "MACHINE_FEATURES:append = \" rk3288w\""
			fi
			echo
		} > build/conf/local.conf

		echo "=========================================="
		echo "          Start building machine($RK_YOCTO_CFG)"
		echo "=========================================="
	fi

	{
		if [ "$RK_WIFIBT_CHIP" ]; then
			echo "include include/wifibt.conf"
		fi

		if [ "$RK_YOCTO_CHROMIUM" ]; then
			echo "include include/browser.conf"
		fi

		echo "include include/rksdk.conf"

		echo

		echo "PREFERRED_VERSION_linux-rockchip :=" \
			"\"$RK_KERNEL_VERSION_REAL%\""
		echo "LINUXLIBCVERSION := \"$RK_KERNEL_VERSION_REAL-custom%\""
		case "$RK_CHIP_FAMILY" in
			px30|rk3326|rk3562|rk3566_rk3568|rk3588)
				echo "MALI_VERSION := \"g13p0\"" ;;
		esac
		echo "DISPLAY_PLATFORM := \"$RK_YOCTO_DISPLAY_PLATFORM\""
	} > build/conf/rksdk_override.conf

	source oe-init-build-env build
	LANG=en_US.UTF-8 LANGUAGE=en_US.en LC_ALL=en_US.UTF-8 \
		bitbake core-image-minimal -f -c rootfs -c image_complete \
		-R conf/rksdk_override.conf

	ln -rsf "$PWD/latest/rootfs.img" $ROOTFS_DIR/rootfs.ext4

	touch "$RK_LOG_DIR/post-rootfs.log"
	cat "$RK_LOG_DIR/post-rootfs.log"
	finish_build build_yocto $@
}

build_debian()
{
	check_config RK_DEBIAN_VERSION || return 0

	ROOTFS_DIR="${1:-$RK_OUTDIR/debian}"
	ARCH=${RK_DEBIAN_ARCH:-armhf}

	"$SCRIPTS_DIR/check-debian.sh"

	echo "=========================================="
	echo "          Start building $RK_DEBIAN_VERSION($ARCH)"
	echo "=========================================="

	cd debian
	if [ ! -f linaro-$RK_DEBIAN_VERSION-alip-*.tar.gz ]; then
		RELEASE=$RK_DEBIAN_VERSION TARGET=desktop ARCH=$ARCH \
			./mk-base-debian.sh
		ln -sf linaro-$RK_DEBIAN_VERSION-alip-*.tar.gz \
			linaro-$RK_DEBIAN_VERSION-$ARCH.tar.gz
	fi

	VERSION=debug ARCH=$ARCH ./mk-rootfs-$RK_DEBIAN_VERSION.sh
	./mk-image.sh

	ln -rsf "$PWD/linaro-rootfs.img" $ROOTFS_DIR/rootfs.ext4

	finish_build build_debian $@
}

# Hooks

usage_hook()
{
	echo -e "buildroot-config[:<config>]       \tmodify buildroot defconfig"
	echo -e "buildroot-make[:<arg1>:<arg2>]    \trun buildroot make (alias bmake)"
	echo -e "rootfs[:<rootfs type>]            \tbuild default rootfs"
	echo -e "buildroot                         \tbuild buildroot rootfs"
	echo -e "yocto                             \tbuild yocto rootfs"
	echo -e "debian                            \tbuild debian rootfs"
}

clean_hook()
{
	rm -rf yocto/build/tmp yocto/build/*cache
	rm -rf debian/binary

	if check_config RK_BUILDROOT_CFG &>/dev/null; then
		rm -rf buildroot/output/$RK_BUILDROOT_CFG
	fi

	rm -rf "$RK_OUTDIR/buildroot"
	rm -rf "$RK_OUTDIR/yocto"
	rm -rf "$RK_OUTDIR/debian"
	rm -rf "$RK_OUTDIR/rootfs"
}

INIT_CMDS="default buildroot debian yocto"
init_hook()
{
	load_config RK_ROOTFS_TYPE
	check_config RK_ROOTFS_TYPE &>/dev/null || return 0

	# Priority: cmdline > custom env
	if [ "$1" != default ]; then
		export RK_ROOTFS_SYSTEM=$1
		echo "Using rootfs system($RK_ROOTFS_SYSTEM) from cmdline"
	elif [ "$RK_ROOTFS_SYSTEM" ]; then
		export RK_ROOTFS_SYSTEM=${RK_ROOTFS_SYSTEM//\"/}
		echo "Using rootfs system($RK_ROOTFS_SYSTEM) from environment"
	else
		return 0
	fi

	ROOTFS_CONFIG="RK_ROOTFS_SYSTEM=\"$RK_ROOTFS_SYSTEM\""
	ROOTFS_UPPER=$(echo $RK_ROOTFS_SYSTEM | tr 'a-z' 'A-Z')
	ROOTFS_CHOICE="RK_ROOTFS_SYSTEM_$ROOTFS_UPPER"
	if ! grep -q "^$ROOTFS_CONFIG$" "$RK_CONFIG"; then
		if ! grep -wq "$ROOTFS_CHOICE" "$RK_CONFIG"; then
			echo -e "\e[35m$RK_ROOTFS_SYSTEM not supported!\e[0m"
			return 0
		fi

		sed -i -e "/RK_ROOTFS_SYSTEM/d" "$RK_CONFIG"
		echo "$ROOTFS_CONFIG" >> "$RK_CONFIG"
		echo "$ROOTFS_CHOICE=y" >> "$RK_CONFIG"
		"$SCRIPTS_DIR/mk-config.sh" olddefconfig &>/dev/null
	fi
}

PRE_BUILD_CMDS="buildroot-config buildroot-make bmake"
pre_build_hook()
{
	check_config RK_ROOTFS_TYPE || return 0

	case "$1" in
		buildroot-make | bmake)
			check_config RK_BUILDROOT_CFG || return 0

			shift
			"$SCRIPTS_DIR/mk-buildroot.sh" $RK_BUILDROOT_CFG make $@
			finish_build buildroot-make $@
			;;
		buildroot-config)
			BUILDROOT_BOARD="${2:-"$RK_BUILDROOT_CFG"}"

			[ "$BUILDROOT_BOARD" ] || return 0

			TEMP_DIR=$(mktemp -d)
			unset BUILDROOT_OUTPUT_DIR
			"$SDK_DIR/buildroot/build/parse_defconfig.sh" \
				"$BUILDROOT_BOARD" "$TEMP_DIR/.config"
			make -C "$SDK_DIR/buildroot" O="$TEMP_DIR" menuconfig
			"$SDK_DIR/buildroot/build/update_defconfig.sh" \
				"$BUILDROOT_BOARD" "$TEMP_DIR"
			rm -rf "$TEMP_DIR"

			finish_build $@
			;;
	esac
}

BUILD_CMDS="rootfs buildroot debian yocto"
build_hook()
{
	check_config RK_ROOTFS_TYPE || return 0

	if [ -z "$1" -o "$1" = rootfs ]; then
		ROOTFS=${RK_ROOTFS_SYSTEM:-buildroot}
	else
		ROOTFS=$1
	fi

	ROOTFS_IMG=rootfs.${RK_ROOTFS_TYPE}
	ROOTFS_DIR="$RK_OUTDIR/rootfs"

	echo "=========================================="
	echo "          Start building rootfs($ROOTFS)"
	echo "=========================================="

	rm -rf "$ROOTFS_DIR"
	mkdir -p "$ROOTFS_DIR"

	case "$ROOTFS" in
		yocto) build_yocto "$ROOTFS_DIR" ;;
		debian) build_debian "$ROOTFS_DIR" ;;
		buildroot) build_buildroot "$ROOTFS_DIR" ;;
		*) usage ;;
	esac

	if [ ! -f "$ROOTFS_DIR/$ROOTFS_IMG" ]; then
		echo "There's no $ROOTFS_IMG generated..."
		exit 1
	fi

	if [ "$RK_ROOTFS_INITRD" ]; then
		/usr/bin/time -f "you take %E to pack initrd image" \
			"$SCRIPTS_DIR/mk-ramdisk.sh" "$ROOTFS_DIR/$ROOTFS_IMG" \
			"$ROOTFS_DIR/boot.img" "$RK_BOOT_FIT_ITS"
		ln -rsf "$ROOTFS_DIR/boot.img" "$RK_FIRMWARE_DIR/boot.img"
	else
		ln -rsf "$ROOTFS_DIR/$ROOTFS_IMG" "$RK_FIRMWARE_DIR/rootfs.img"
	fi

	finish_build build_rootfs $@
}

source "${BUILD_HELPER:-$(dirname "$(realpath "$0")")/../build-hooks/build-helper}"

case "${1:-rootfs}" in
	buildroot-config | buildroot-make | bmake) pre_build_hook $@ ;;
	buildroot | debian | yocto) init_hook $@ ;&
	*) build_hook $@ ;;
esac
