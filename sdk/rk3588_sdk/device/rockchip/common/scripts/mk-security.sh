#!/bin/bash -e

BOOT_FIXED_CONFIGS=" \
	CONFIG_BLK_DEV_DM \
	CONFIG_DM_CRYPT \
	CONFIG_BLK_DEV_CRYPTOLOOP \
	CONFIG_DM_VERITY"

BOOT_OPTEE_FIXED_CONFIGS=" \
	CONFIG_TEE \
	CONFIG_OPTEE"

UBOOT_FIXED_CONFIGS=" \
	CONFIG_FIT_SIGNATURE \
	CONFIG_SPL_FIT_SIGNATURE"

UBOOT_AB_FIXED_CONFIGS=" \
	CONFIG_ANDROID_AB"

ROOTFS_UPDATE_ENGINEBIN_CONFIGS=" \
	BR2_PACKAGE_RECOVERY \
	BR2_PACKAGE_RECOVERY_UPDATEENGINEBIN"

ROOTFS_AB_FIXED_CONFIGS=" \
	$ROOTFS_UPDATE_ENGINEBIN_CONFIGS \
	BR2_PACKAGE_RECOVERY_BOOTCONTROL"

defconfig_check()
{
	# 1. defconfig 2. fixed config
	echo debug-$1
	for i in $2
	do
		echo "look for $i"
		result=$(cat $1 | grep "${i}=y" -w || echo "No found")
		if [ "$result" = "No found" ]; then
			echo -e "\e[41;1;37mSecurity: No found config ${i} in $1 \e[0m"
			echo "make sure your config include this list"
			echo "---------------------------------------"
			echo "$2"
			echo "---------------------------------------"
			return 1;
		fi
	done
	return 0
}

find_string_in_config()
{
	result=$(cat "$2" | grep "$1" || echo "No found")
	if [ "$result" = "No found" ]; then
		echo "Security: No found string $1 in $2"
		return 1;
	fi
	return 0;
}

security_check()
{
	[ "$RK_SECURITY" ] || return 0

	if [ ! -d u-boot/keys ]; then
		echo "ERROR: No root keys(u-boot/keys) found in u-boot"
		echo "       Create it by ./build.sh createkeys or move your key to it"
		return 1
	fi

	if [ "$RK_SECURITY_CHECK_METHOD" = "DM-E" ]; then
		if [ ! -f u-boot/keys/root_passwd ]; then
			echo "ERROR: No root passwd(u-boot/keys/root_passwd) found in u-boot"
			echo "       echo your root key for sudo to u-boot/keys/root_passwd"
			echo "       some operations need supper user permission when create encrypt image"
			return 1
		fi

		if [ ! -f u-boot/keys/system_enc_key ]; then
			echo "ERROR: No enc key(u-boot/keys/system_enc_key) found in u-boot"
			echo "       Create it by ./build.sh createkeys or move your key to it"
			return 1
		fi

		BOOT_FIXED_CONFIGS="$BOOT_FIXED_CONFIGS $BOOT_OPTEE_FIXED_CONFIGS"
	fi

	echo "check kernel defconfig"
	defconfig_check \
		kernel/arch/$RK_KERNEL_ARCH/configs/$RK_KERNEL_CFG \
		"$BOOT_FIXED_CONFIGS"

	if [ -n "$RK_AB_UPDATE" ]; then
		UBOOT_FIXED_CONFIGS="$UBOOT_FIXED_CONFIGS \
			$UBOOT_AB_FIXED_CONFIGS"

		defconfig_check \
			buildroot/configs/${RK_BUILDROOT_CFG}_defconfig \
			"$ROOTFS_AB_FIXED_CONFIGS"
	fi
	echo "check uboot defconfig"
	defconfig_check u-boot/configs/${RK_UBOOT_CFG}_defconfig \
		"$UBOOT_FIXED_CONFIGS"

	if [ "$RK_SECURITY_CHECK_METHOD" = "DM-E" ]; then
		echo "check ramdisk defconfig"
		defconfig_check \
			buildroot/configs/${RK_SECURITY_INITRD_CFG}_defconfig \
			"$ROOTFS_UPDATE_ENGINEBIN_CONFIGS"
	fi

	echo "check rootfs defconfig"
	find_string_in_config "security-system-overlay" \
		"buildroot/configs/${RK_BUILDROOT_CFG}_defconfig"

	echo "Security: finish check"
}

build_security_keys()
{
	if [ -d u-boot/keys ]; then
		echo "ERROR: u-boot/keys already exists"
		return 1
	fi

	mkdir -p u-boot/keys
	cd u-boot/keys
	"$SDK_DIR/rkbin/tools/rk_sign_tool" kk --bits 2048 --out ./

	ln -rsf private_key.pem dev.key
	ln -rsf public_key.pem dev.pubkey

	cd "$SDK_DIR"

	openssl req -batch -new -x509 -key u-boot/keys/dev.key \
		-out u-boot/keys/dev.crt

	openssl rand -out u-boot/keys/system_enc_key -hex 32
}

build_security_ramboot()
{
	check_config RK_SECURITY_INITRD_CFG || return 0

	echo "=========================================="
	echo "          Start building security ramboot(buildroot)"
	echo "=========================================="

	DST_DIR="$RK_OUTDIR/security-ramboot"

	if [ ! -r "$RK_FIRMWARE_DIR/rootfs.img" ]; then
		echo "Rootfs is not ready, building it for security..."
		"$SCRIPTS_DIR/mk-rootfs.sh"
	fi

	# Prepare misc and initrd overlay with rootfs.img
	"$SCRIPTS_DIR/mk-dm.sh" $RK_SECURITY_CHECK_METHOD \
		"$RK_FIRMWARE_DIR/rootfs.img"

	/usr/bin/time -f "you take %E to build security initrd(buildroot)" \
		"$SCRIPTS_DIR/mk-buildroot.sh" $RK_SECURITY_INITRD_CFG \
		"$DST_DIR"

	/usr/bin/time -f "you take %E to pack security ramboot image" \
		"$SCRIPTS_DIR/mk-ramdisk.sh" \
		"$DST_DIR/rootfs.$RK_SECURITY_INITRD_TYPE" \
		"$DST_DIR/ramboot.img" "$RK_SECURITY_FIT_ITS"

	ln -rsf "$DST_DIR/ramboot.img" "$RK_FIRMWARE_DIR/boot.img"

	finish_build $@
}

# Hooks

usage_hook()
{
	echo -e "security_check                    \tcheck contidions for security boot"
	echo -e "createkeys                        \tbuild security boot keys"
	echo -e "security_ramboot                  \tbuild security ramboot"
	echo -e "security_uboot                    \tbuild uboot with security"
	echo -e "security_boot                     \tbuild boot with security"
	echo -e "security_recovery                 \tbuild recovery with security"
	echo -e "security_rootfs                   \tbuild rootfs with security"
}

BUILD_CMDS="security_check createkeys security_ramboot security_uboot \
	security_boot security_recovery security_rootfs"
build_hook()
{
	check_config RK_SECURITY || return 0

	case "${1:-security_ramboot}" in
		security_check) security_check ;;
		createkeys) build_security_keys ;;
		security_ramboot) build_security_ramboot ;;
		security_uboot) "$SCRIPTS_DIR"/mk-loader.sh uboot ;;
		security_boot)
			"$SCRIPTS_DIR"/mk-kernel.sh
			build_security_ramboot
			"$SCRIPTS_DIR"/mk-loader.sh uboot boot
			;;
		security_recovery)
			check_config RK_RECOVERY_CFG || return 0
			"$SCRIPTS_DIR"/mk-recovery.sh
			"$SCRIPTS_DIR"/mk-loader.sh uboot recovery
			;;
		security_rootfs)
			"$SCRIPTS_DIR"/mk-rootfs.sh
			build_security_ramboot
			"$SCRIPTS_DIR"/mk-loader.sh uboot boot
			;;
		*) usage ;;
	esac
}

source "${BUILD_HELPER:-$(dirname "$(realpath "$0")")/../build-hooks/build-helper}"

build_hook $@
