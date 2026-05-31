#!/bin/bash -e

[ -z "$RK_EXTRA_FONTS_DISABLED" ] || exit 0
if [ "$RK_EXTRA_FONTS_DEFAULT" -a "$POST_OS" != yocto ]; then
	echo -e "\e[33mNo extra fonts for $POST_OS by default\e[0m"
	exit 0
fi

TARGET_DIR="$1"
[ "$TARGET_DIR" ] || exit 1

OVERLAY_DIR="$(dirname "$(realpath "$0")")"
cd "$OVERLAY_DIR"

for f in *.tar; do
	echo "Installing extra font(${f%.tar}) to $TARGET_DIR..."
	tar xf "$f" -C "$TARGET_DIR"
done
