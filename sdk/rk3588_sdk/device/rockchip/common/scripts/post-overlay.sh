#!/bin/bash -e

source "${POST_HELPER:-$(dirname "$(realpath "$0")")/../post-hooks/post-helper}"

RK_RSYNC="rsync -av --chmod=u=rwX,go=rX --copy-unsafe-links --exclude .empty"

cd "$COMMON_DIR/overlays"

install_overlay()
{
	OVERLAY="$1"
	[ -d "$OVERLAY" ] || return 0

	if [ "$RK_OVERLAY_ALLOWED" ]; then
		for d in $RK_OVERLAY_ALLOWED; do
			basename "$d"
		done | grep -q "^$(basename $OVERLAY)$" || return 0
	fi

	OVERLAY="$(realpath "$OVERLAY")"
	if [ -x "$OVERLAY/install.sh" ]; then
		echo -ne "\e[36m"
		echo "Handling overlay: $OVERLAY)..."
		echo -ne "\e[0m"
		RK_RSYNC="$RK_RSYNC" \
			"$OVERLAY/install.sh" "$TARGET_DIR" "$POST_OS"
	else
		echo -ne "\e[36m"
		echo "Installing overlay: $OVERLAY to $TARGET_DIR..."
		echo -ne "\e[0m"
		$RK_RSYNC "$OVERLAY/" "$TARGET_DIR/"
	fi
}

# No overlays for rootfs without RK_ROOTFS_OVERLAY_DIRS
[ "$POST_ROOTFS" -a "$RK_ROOTFS_OVERLAY_DIRS" ] || exit 0

install_overlay common

# No rootfs overlays for non-rootfs
[ "$POST_ROOTFS" ] || exit 0

install_overlay $POST_OS

for overlay in $(find rootfs/ -mindepth 1 -maxdepth 1 -type d); do
	install_overlay $overlay
done

for overlay in $RK_ROOTFS_EXTRA_OVERLAY_DIRS; do
	install_overlay "$overlay"
done
