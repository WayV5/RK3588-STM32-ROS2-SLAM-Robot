#!/bin/bash -e

build_uefi()
{
	check_config RK_KERNEL_DTB || return 1

	if [ "$RK_CHIP" != rk3588 -o ! -d uefi ]; then
		echo "UEFI not supported!"
		return 1
	fi

	if [ ! -f "$RK_KERNEL_DTB" ]; then
		echo "$RK_KERNEL_DTB not exists!"
		return 1
	fi

	UEFI_DIR=uefi/edk2-platforms/Platform/Rockchip/DeviceTree

	run_command cp "$RK_KERNEL_DTB" $UEFI_DIR/$RK_CHIP.dtb
	run_command cd uefi
	run_command $UMAKE $RK_UBOOT_CFG
}

build_uboot()
{
	check_config RK_UBOOT_CFG || return 0

	if [ -z "$DRY_RUN" ]; then
		rm -f u-boot/*.bin u-boot/*.img
	fi

	UARGS_COMMON="$RK_UBOOT_OPTS \
		${RK_UBOOT_INI:+../rkbin/RKBOOT/$RK_UBOOT_INI} \
		${RK_UBOOT_TRUST_INI:+../rkbin/RKTRUST/$RK_UBOOT_TRUST_INI}"
	UARGS="$UARGS_COMMON ${RK_UBOOT_SPL:+--spl-new} \
		${RK_SECURITY_BURN_KEY:+--burn-key-hash}"

	if [ "$RK_SECURITY" ]; then
		for IMAGE in ${1:-boot.img ${RK_RECOVERY_CFG:+recovery.img}}; do
			[ "$DRY_RUN" ] || \
				cp "$RK_FIRMWARE_DIR/$IMAGE" u-boot/

			UARGS="$UARGS --${IMAGE/./_} $SDK_DIR/u-boot/$IMAGE"
		done
	fi

	run_command cd u-boot
	run_command $UMAKE $RK_UBOOT_CFG $RK_UBOOT_CFG_FRAGMENTS $UARGS

	if [ "$RK_UBOOT_SPL" ]; then
		if [ "$DRY_RUN" ] || \
			! grep -q "ROCKCHIP_FIT_IMAGE_PACK=y" .config; then
			# Repack SPL for non-FIT u-boot
			run_command $UMAKE $UARGS_COMMON --spl
		fi
	fi

	if [ "$RK_UBOOT_RAW" ]; then
		run_command $UMAKE $UARGS_COMMON --idblock
	fi

	run_command cd ..

	if [ "$DRY_RUN" ]; then
		return 0
	fi

	if [ "$RK_SECURITY" ];then
		for IMAGE in u-boot/boot.img u-boot/recovery.img; do
			[ ! -r $IMAGE ] || \
				ln -rsf $IMAGE "$RK_SECURITY_FIRMWARE_DIR"
		done
	fi

	LOADER="$(echo u-boot/*_loader_*.bin | head -1)"
	ln -rsf "$LOADER" "$RK_FIRMWARE_DIR"/MiniLoaderAll.bin

	ln -rsf u-boot/uboot.img "$RK_FIRMWARE_DIR"
	[ ! -e u-boot/trust.img ] || \
		ln -rsf u-boot/trust.img "$RK_FIRMWARE_DIR"
}

# Hooks

usage_hook()
{
	echo -e "loader[:cmds]                    \tbuild loader (uboot)"
	echo -e "uboot[:cmds]                     \tbuild u-boot"
	echo -e "uefi[:cmds]                      \tbuild uefi"
}

clean_hook()
{
	make -C u-boot distclean
}

BUILD_CMDS="loader uboot uefi"
build_hook()
{
	if echo $RK_UBOOT_CFG $RK_UBOOT_CFG_FRAGMENTS | grep -q aarch32 && \
		[ "$RK_UBOOT_ARCH" = arm64 ]; then
		echo -e "\e[31mWrong u-boot arch ($RK_UBOOT_ARCH) for config:" \
			"$RK_UBOOT_CFG $RK_UBOOT_CFG_FRAGMENTS\n\e[0m"
		export RK_UBOOT_ARCH=arm
	fi

	RK_UBOOT_TOOLCHAIN="$(get_toolchain "$RK_UBOOT_ARCH")"

	echo "Toolchain for loader (u-boot):"
	echo "${RK_UBOOT_TOOLCHAIN:-gcc}"
	echo

	export UMAKE="./make.sh CROSS_COMPILE=$RK_UBOOT_TOOLCHAIN"

	if [ "$DRY_RUN" ]; then
		echo -e "\e[35mCommands of building $1:\e[0m"
	else
		echo "=========================================="
		echo "          Start building $1"
		echo "=========================================="
	fi

	TARGET="$1"
	shift
	[ "$1" != cmds ] || shift

	case "$TARGET" in
		uboot | loader) build_uboot $@ ;;
		uefi) build_uefi $@ ;;
		*) usage ;;
	esac

	if [ -z "$DRY_RUN" ]; then
		finish_build build_$TARGET $@
	fi
}

build_hook_dry()
{
	DRY_RUN=1 build_hook $@
}

source "${BUILD_HELPER:-$(dirname "$(realpath "$0")")/../build-hooks/build-helper}"

build_hook ${@:-loader}
