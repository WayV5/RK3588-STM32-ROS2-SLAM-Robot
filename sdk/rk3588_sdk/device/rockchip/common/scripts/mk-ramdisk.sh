#!/bin/bash -e

RAMDISK_IMG="$1"
TARGET_IMG="$2"
ITS="$3"

if [ ! -f "$RAMDISK_IMG" ]; then
	echo "$RAMDISK_IMG doesn't exist"
	exit 0
fi

KERNEL_IMG="$RK_KERNEL_IMG"

if [ ! -f "$KERNEL_IMG" ]; then
	echo "Build kernel for initrd"
	"$SCRIPTS_DIR/mk-kernel.sh"
fi

if echo $RAMDISK_IMG | grep -q ".romfs$"; then
	cat "$RAMDISK_IMG" | gzip -n -f -9 > "$RAMDISK_IMG.gz"
	cat "$KERNEL_IMG" | gzip -n -f -9 > "$KERNEL_IMG.gz"
	RAMDISK_IMG="$RAMDISK_IMG.gz"
	KERNEL_IMG="$KERNEL_IMG.gz"
fi

echo "Packing $RAMDISK_IMG to $TARGET_IMG"
if [ -n "$ITS" ]; then
	"$SCRIPTS_DIR/mk-fitimage.sh" "$TARGET_IMG" "$ITS" \
		"$KERNEL_IMG" "$RAMDISK_IMG"
else
	kernel/scripts/mkbootimg --kernel "$KERNEL_IMG" \
		--ramdisk "$RAMDISK_IMG" --second "kernel/resource.img" \
		-o "$TARGET_IMG"
fi
