#!/bin/bash
# RK3588 构建脚本
# Usage: ./build.sh uboot | kernel | rootfs
set -e

SDK="$(cd "$(dirname "$0")" && pwd)/rk3588_sdk"
CROSS_COMPILE="$SDK/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-"
ROOTFS_DIR="$(cd "$(dirname "$0")" && pwd)/rootfs"
TARGETS="$(cd "$(dirname "$0")" && pwd)/../targets/rk3588"

mkdir -p "$TARGETS"

usage() {
    echo "Usage: $0 {uboot|kernel|rootfs}"
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
    ROOTFS_SIZE_MB=$(sudo du -sm "$ROOTFS_DIR" | cut -f1)
    IMG_SIZE_MB=$((ROOTFS_SIZE_MB + 512))
    ROOTFS_IMG="$TARGETS/rootfs.img"

    rm -f "$ROOTFS_IMG"
    echo "rootfs: ${ROOTFS_SIZE_MB}MB → 镜像 ${IMG_SIZE_MB}MB"

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

else
    usage
fi
