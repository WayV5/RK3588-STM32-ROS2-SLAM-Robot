#!/bin/bash -e

POST_ROOTFS_ONLY=1

source "${POST_HELPER:-$(dirname "$(realpath "$0")")/../post-hooks/post-helper}"

echo "Preparing extra partitions..."

for idx in $(seq 1 "$(rk_extra_part_num)"); do
	MOUNTPOINT="$(rk_extra_part_mountpoint $idx)"
	FAKEROOT_SCRIPT="$(rk_extra_part_fakeroot_script $idx)"
	OUTDIR="$(rk_extra_part_outdir $idx)"
	MOUNT_DIR="$(rk_extra_part_mount_dir $idx)"

	rm -rf "$FAKEROOT_SCRIPT" "$OUTDIR" "$MOUNT_DIR"
	mkdir -p "$TARGET_DIR/$MOUNTPOINT"
	ln -rsf "$TARGET_DIR/$MOUNTPOINT" "$MOUNT_DIR"

	if rk_extra_part_builtin $idx; then
		rk_extra_part_prepare $idx
		echo "Merging $OUTDIR into $TARGET_DIR/$MOUNTPOINT (built-in)"
		rsync -a "$OUTDIR/" "$TARGET_DIR/$MOUNTPOINT/"
	fi
done
