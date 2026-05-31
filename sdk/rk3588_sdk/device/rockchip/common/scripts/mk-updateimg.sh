#!/bin/bash -e

RK_PACK_TOOL_DIR="$SDK_DIR/tools/linux/Linux_Pack_Firmware/rockdev"

gen_package_file()
{
	TYPE="${1:-update}"
	PARAMETER="${2:-parameter.txt}"
	PKG_FILE="${3:-package-file}"

	if [ ! -r "$PARAMETER" ]; then
		echo -e "\e[31munable to parse $PARAMETER\e[0m"
		exit 1
	fi

	{
		echo -e "# NAME\tPATH"
		echo -e "package-file\tpackage-file"
		echo -e "parameter\tparameter.txt"
	} > "$PKG_FILE"

	if [ "$TYPE" = template -o -r MiniLoaderAll.bin ]; then
		echo -e "bootloader\tMiniLoaderAll.bin" >> "$PKG_FILE"
	fi

	for part in $(rk_partition_parse_names "$PARAMETER"); do
		if echo $part | grep -q "_b$"; then
			# Not packing *_b partition for ota
			if [ "$TYPE" = ota ]; then
				continue
			fi
		fi

		case $part in
			backup)
				echo -e "backup\tRESERVED" >> "$PKG_FILE"
				continue
				;;
			system|system_[ab]) IMAGE=rootfs.img ;;
			*_a) IMAGE="${part%_a}.img" ;;
			*_b) IMAGE="${part%_b}.img" ;;
			*) IMAGE="$part.img" ;;
		esac

		if [ "$TYPE" = template -o -r "$IMAGE" ]; then
			echo -e "$part\t$IMAGE" >> "$PKG_FILE"
		fi
	done
}

build_updateimg()
{
	check_config RK_UPDATE || return 0

	TARGET="${1:-$RK_ROCKDEV_DIR/update.img}"
	TYPE="${2:-update}"
	PKG_FILE="${3:-$RK_PACKAGE_FILE}"
	OUT_DIR="$RK_OUTDIR/$TYPE"
	IMAGE_DIR="$OUT_DIR/Image"

	# Make sure that the firmware is ready
	if [ ! -r "$RK_ROCKDEV_DIR/parameter.txt" ]; then
		echo "Firmware is not ready, building it..."
		"$SCRIPTS_DIR/mk-firmware.sh"
	fi

	# Make sure that the loader is ready
	if [ ! -r "$RK_ROCKDEV_DIR/MiniLoaderAll.bin" ]; then
		echo "Loader is not ready, building it..."
		"$SCRIPTS_DIR/mk-loader.sh"
	fi

	echo "=========================================="
	echo "          Start packing $2 update image"
	echo "=========================================="

	rm -rf "$TARGET" "$OUT_DIR"
	mkdir -p "$IMAGE_DIR"
	cd "$IMAGE_DIR"

	# Prepare images
	ln -rsf "$RK_ROCKDEV_DIR"/* .
	rm -f update.img

	# Prepare package-file
	if [ "$PKG_FILE" ]; then
		PKG_FILE="$CHIP_DIR/$PKG_FILE"
		if [ ! -r "$PKG_FILE" ]; then
			echo "$PKG_FILE not exists!"
			exit 1
		fi
		ln -rsf "$PKG_FILE" package-file
	else
		echo "Generating package-file for $TYPE:"
		gen_package_file $TYPE
		cat package-file
	fi

	echo "Packing $TARGET for $TYPE..."

	if [ ! -r MiniLoaderAll.bin ]; then
		echo -e "\e[31mMiniLoaderAll.bin is missing\e[0m"
		exit 1
	fi

	TAG=RK$(hexdump -s 21 -n 4 -e '4 "%c"' MiniLoaderAll.bin | rev)
	"$RK_PACK_TOOL_DIR/afptool" -pack ./ update.raw.img
	"$RK_PACK_TOOL_DIR/rkImageMaker" -$TAG MiniLoaderAll.bin \
		update.raw.img update.img -os_type:androidos

	ln -rsf "$IMAGE_DIR/package-file" "$OUT_DIR"
	ln -rsf "$IMAGE_DIR/update.img" "$OUT_DIR"
	ln -rsf "$IMAGE_DIR/update.img" "$TARGET"

	finish_build build_updateimg $@
}

build_ota_updateimg()
{
	check_config RK_AB_UPDATE || return 0

	echo "Make A/B update image for OTA"

	build_updateimg "$RK_ROCKDEV_DIR/update_ota.img" ota \
		$RK_OTA_PACKAGE_FILE

	finish_build
}

build_ab_updateimg()
{
	check_config RK_AB_UPDATE || return 0

	build_ota_updateimg

	echo "Make A/B update image"

	build_updateimg "$RK_ROCKDEV_DIR/update_ab.img" ab

	finish_build
}

# Hooks

usage_hook()
{
	echo -e "edit-package-file                 \tedit package-file"
	echo -e "edit-ota-package-file             \tedit A/B OTA package-file"
	echo -e "updateimg                         \tbuild update image"
	echo -e "otapackage                        \tbuild A/B OTA update image"
}

clean_hook()
{
	rm -rf "$RK_OUTDIR/update"
	rm -rf "$RK_OUTDIR/ota"
	rm -rf "$RK_OUTDIR/ab"
	rm -rf "$RK_FIRMWARE_DIR/*update.img"
	rm -rf "$RK_ROCKDEV_DIR/*update.img"
}

INIT_CMDS="edit-package-file edit-ota-package-file"
init_hook()
{
	case "$1" in
		edit-package-file)
			BASE_CFG=RK_PACKAGE_FILE
			PKG_FILE="$CHIP_DIR/package-file"
			;;
		edit-ota-package-file)
			BASE_CFG=RK_AB_OTA_PACKAGE_FILE
			PKG_FILE="$CHIP_DIR/ab-ota-package-file"
			;;
		*) return 0 ;;
	esac

	load_config $BASE_CFG
	if ! check_config $BASE_CFG &>/dev/null; then
		sed -i '/$BASE_CFG/d' "$RK_CONFIG"
		echo "${BASE_CFG}_CUSTOM=y" >> "$RK_CONFIG"
		echo "$BASE_CFG=$PKG_FILE" >> "$RK_CONFIG"
                "$SCRIPTS_DIR/mk-config.sh" olddefconfig &>/dev/null
                "$SCRIPTS_DIR/mk-config.sh" savedefconfig &>/dev/null
	fi
}

PRE_BUILD_CMDS="edit-package-file edit-ota-package-file"
pre_build_hook()
{
	case "$1" in
		edit-package-file)
			check_config RK_PACKAGE_FILE || return 0
			PKG_FILE="$CHIP_DIR/$RK_PACKAGE_FILE" ;;
		edit-ota-package-file)
			check_config RK_AB_OTA_PACKAGE_FILE || return 0
			PKG_FILE="$CHIP_DIR/$RK_AB_OTA_PACKAGE_FILE"
			;;
		*) return 0 ;;
	esac

	PKG_FILE="$(realpath "$PKG_FILE")"
	if [ ! -r "$PKG_FILE" ]; then
		echo "Generating template $PKG_FILE"
		gen_package_file template "$CHIP_DIR/$RK_PARAMETER" "$PKG_FILE"
	fi
	eval ${EDITOR:-vi} "$PKG_FILE"

	finish_build $@
}

POST_BUILD_CMDS="updateimg otapackage"
post_build_hook()
{
	case "$1" in
		updateimg)
			if [ "$RK_AB_UPDATE" ]; then
				build_ab_updateimg
			else
				build_updateimg
			fi
			;;
		otapackage) build_ota_updateimg ;;
		*) usage ;;
	esac
}

source "${BUILD_HELPER:-$(dirname "$(realpath "$0")")/../build-hooks/build-helper}"

case "$@" in
	edit-package-file|edit-ota-package-file)
		init_hook $@
		pre_build_hook $@
		;;
	*) post_build_hook ${@:-updateimg} ;;
esac
