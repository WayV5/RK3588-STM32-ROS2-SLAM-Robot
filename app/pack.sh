#!/bin/bash
# app/pack.sh — 打包 app 源码为 ext4 镜像, 在 RK3588 上原生编译
#
# 只打包 src/ 源码和空白的 build/install/log 目录.
# colcon build 在 RK3588 板子上执行 (ARM64 原生编译).
#
# Usage:
#   ./pack.sh                打包 app.img
#
# 输出:
#   targets/rk3588/app.img    ext4 镜像 (root:root, 包含 src/ + 空 build/install/log/)
#
# 目标板分区: /dev/mmcblk0p8 → /app
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGETS="$PROJECT_DIR/targets/rk3588"
IMG_FILE="$TARGETS/app.img"
APP_MOUNT="/app"
APP_PARTITION="/dev/mmcblk0p8"

echo "===== 打包 app.img ====="

if [ ! -d "$SCRIPT_DIR/src" ]; then
	echo "错误: src/ 目录不存在"
	exit 1
fi

# 估算源码大小
SRC_SIZE_KB=$(du -sk "$SCRIPT_DIR/src" | cut -f1)
SRC_SIZE_MB=$(((SRC_SIZE_KB + 1023) / 1024))
IMG_SIZE_MB=$((SRC_SIZE_MB + 64))  # 留 64MB 给编译产物

rm -f "$IMG_FILE"

echo "  src/: ${SRC_SIZE_MB}MB → 镜像 ${IMG_SIZE_MB}MB"

# 创建 ext4 镜像
sudo dd if=/dev/zero of="$IMG_FILE" bs=1M count="$IMG_SIZE_MB" status=progress
sudo mkfs.ext4 -F -L robot_app "$IMG_FILE"

# 挂载 → 复制源码 + 创建空编译目录
MNT_DIR=$(mktemp -d)
sudo mount "$IMG_FILE" "$MNT_DIR"

echo "  复制 src/ → 镜像..."
sudo cp -a "$SCRIPT_DIR/src" "$MNT_DIR/"

# 创建空的编译目录 (板子上 colcon build 会填充)
sudo mkdir -p "$MNT_DIR/build" "$MNT_DIR/install" "$MNT_DIR/log"

# 权限
sudo chown -R root:root "$MNT_DIR"/
sudo find "$MNT_DIR" -type d -exec chmod 755 {} \;
sudo find "$MNT_DIR" -type f -exec chmod 644 {} \;

sudo umount "$MNT_DIR"
rmdir "$MNT_DIR"

sudo e2fsck -fy "$IMG_FILE"
sudo resize2fs -M "$IMG_FILE"

echo ""
echo "  → $IMG_FILE ($(du -sh "$IMG_FILE" | cut -f1))"
echo ""

echo "===== 板子操作 ====="
echo ""
echo "  # 1. 烧录 app.img 到 userdata 分区"
echo "  scp $IMG_FILE root@192.168.137.10:/tmp/"
echo "  ssh root@192.168.137.10 \"dd if=/tmp/app.img of=$APP_PARTITION bs=4M && mount $APP_MOUNT\""
echo ""
echo "  # 2. 在板子上编译 (ARM64 原生)"
echo "  ssh root@192.168.137.10"
echo "  cd $APP_MOUNT && source /opt/ros/humble/setup.bash && colcon build --symlink-install"
echo ""
echo "  # 3. 加载环境并运行"
echo "  source $APP_MOUNT/install/setup.bash"
echo "  ros2 run robot_can_gateway can_gateway_node"
echo ""
echo "===== 完成 ====="
