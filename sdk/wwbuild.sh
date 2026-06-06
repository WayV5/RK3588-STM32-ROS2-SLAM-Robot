#!/bin/bash
# RK3588 构建脚本
# Usage: ./wwbuild.sh uboot | kernel | rootfs | wifidriver
set -e

SDK="$(cd "$(dirname "$0")" && pwd)/rk3588_sdk"
CROSS_COMPILE="$SDK/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-"
ROOTFS_DIR="$(cd "$(dirname "$0")" && pwd)/rootfs"
TARGETS="$(cd "$(dirname "$0")" && pwd)/../targets/rk3588"
OVERLAY="$TARGETS/rootfs"

mkdir -p "$TARGETS" "$OVERLAY"

usage() {
	echo "Usage: $0 {uboot|kernel|rootfs|wifidriver}"
	exit 1
}

[ $# -eq 1 ] || usage

# ---- U-Boot ----
if [ "$1" = "uboot" ]; then
	echo "===== 编译 U-Boot ====="
	cd "$SDK/u-boot"
	make CROSS_COMPILE="$CROSS_COMPILE" rk3588_defconfig
	./make.sh CROSS_COMPILE="$CROSS_COMPILE" rk3588

	ln -sf "$SDK/u-boot/uboot.img" "$TARGETS/uboot.img"
	echo "  → $TARGETS/uboot.img"

	LOADER=$(ls "$SDK/u-boot"/*_loader_*.bin 2>/dev/null | head -1)
	if [ -n "$LOADER" ]; then
		ln -sf "$LOADER" "$TARGETS/$(basename $LOADER)"
		echo "  → $TARGETS/$(basename $LOADER)"
	fi
	echo "U-Boot 完成"

# ---- 内核 ----
elif [ "$1" = "kernel" ]; then
	echo "===== 编译内核 ====="
	cd "$SDK/kernel"
	./make.sh board=ATK_DLRK3588

	ln -sf "$SDK/kernel/boot.img" "$TARGETS/boot.img"
	echo "  → $TARGETS/boot.img"
	echo "内核完成"

# ---- Rootfs ----
elif [ "$1" = "rootfs" ]; then
	if [ ! -d "$ROOTFS_DIR" ]; then
		echo "错误: rootfs 目录不存在: $ROOTFS_DIR"
		echo "请先按构建指南 Section 3 制作 rootfs"
		exit 1
	fi

	echo "===== 打包 rootfs ====="

	# 覆盖 overlay 文件到 rootfs
	if [ -d "$OVERLAY" ] && [ "$(ls -A "$OVERLAY" 2>/dev/null)" ]; then
		echo "  overlay: $OVERLAY/ → $ROOTFS_DIR/"
		sudo rsync -a "$OVERLAY/" "$ROOTFS_DIR/"
	fi

	ROOTFS_SIZE_MB=$(sudo du -sm "$ROOTFS_DIR" | cut -f1)
	IMG_SIZE_MB=$((ROOTFS_SIZE_MB + 512))
	ROOTFS_IMG="$TARGETS/rootfs.img"

	rm -f "$ROOTFS_IMG"
	echo "  rootfs: ${ROOTFS_SIZE_MB}MB → 镜像 ${IMG_SIZE_MB}MB"

	sudo dd if=/dev/zero of="$ROOTFS_IMG" bs=1M count="$IMG_SIZE_MB" status=progress
	sudo mkfs.ext4 -F "$ROOTFS_IMG"

	MNT_DIR=$(mktemp -d)
	sudo mount "$ROOTFS_IMG" "$MNT_DIR"
	sudo cp -a "$ROOTFS_DIR"/* "$MNT_DIR"/
	sudo umount "$MNT_DIR"
	rmdir "$MNT_DIR"

	sudo e2fsck -fy "$ROOTFS_IMG"
	sudo resize2fs -M "$ROOTFS_IMG"

	echo "  → $TARGETS/rootfs.img ($(du -sh "$ROOTFS_IMG" | cut -f1))"
	echo "Rootfs 完成"

# ---- WiFi Driver (RTL8733BU) ----
elif [ "$1" = "wifidriver" ]; then
	echo "===== 交叉编译 RTL8733BU WiFi 驱动 ====="
	DRV="$SDK/external/rkwifibt/drivers/rtl8733bu"
	FW_SRC="$SDK/external/rkwifibt/firmware/realtek/RTL8733BU"
	KVER="5.10.160"

	# usr/lib/ 而非 lib/ — Ubuntu 22.04 的 lib 是 usr/lib 的软链接
	MOD_DST="$OVERLAY/usr/lib/modules/$KVER/extra"
	FW_DST="$OVERLAY/usr/lib/firmware"

	# 1) 编译内核模块
	cd "$DRV"
	make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE" KSRC="$SDK/kernel" -j"$(nproc)"
	echo "  → 8733bu.ko"

	# 2) 输出到 targets/rk3588/rootfs/ (staging)
	sudo mkdir -p "$MOD_DST" "$FW_DST"
	sudo cp "$DRV/8733bu.ko" "$MOD_DST/"
	sudo cp "$FW_SRC/"* "$FW_DST/"

	# 3) depmod
	sudo depmod -b "$OVERLAY" "$KVER" 2>/dev/null || {
		echo "  ⚠ depmod 失败 (跨架构), 稍后 rootfs 打包时再试"
	}

	echo "  → $MOD_DST/8733bu.ko"
	echo "  → $FW_DST/"
	echo "  → depmod → $OVERLAY/usr/lib/modules/$KVER/"
	echo ""
	echo "  [!] 请执行  ./wwbuild.sh rootfs  来打包进镜像"
	echo "WiFi 驱动完成"

else
	usage
fi
