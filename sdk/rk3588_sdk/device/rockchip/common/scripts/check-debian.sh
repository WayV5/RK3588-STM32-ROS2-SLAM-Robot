#!/bin/bash -e

SCRIPTS_DIR="${SCRIPTS_DIR:-$(dirname "$(realpath "$0")")}"
RK_DEBIAN_ARCH="${RK_DEBIAN_ARCH:-arm64}"
RK_DATA_DIR="${RK_DATA_DIR:-"$SCRIPTS_DIR/../data/"}"

if [ ! -e "/usr/share/live/build/data/debian-cd/$RK_DEBIAN_VERSION" ]; then
	echo -e "\e[35m"
	echo "Your live-build doesn't support $RK_DEBIAN_VERSION"
	echo "Please replace it:"
	echo "sudo apt-get remove live-build"
	echo "git clone https://salsa.debian.org/live-team/live-build.git --depth 1 -b debian/1%20230131"
	echo "cd live-build"
	echo "rm -rf manpages/po/"
	echo "sudo make install -j8"
	echo -e "\e[0m"
	exit 1
fi

# The debian SDK's e2fsprogs doesn't support new features like
# metadata_csum_seed and orphan_file
if grep -wq metadata_csum_seed /etc/mke2fs.conf; then
	echo -e "\e[35m"
	echo "Your mke2fs is too new: $(mke2fs -V 2>&1 | head -n 1)"
	echo "Please downgrade it:"
	"$SCRIPTS_DIR/install-e2fsprogs.sh"
	echo -e "\e[0m"
	exit 1
fi

case "$RK_DEBIAN_ARCH" in
	arm64) QEMU_ARCH=aarch64 ;;
	armhf) QEMU_ARCH=arm ;;
esac
QEMU_VERSION=$(qemu-$QEMU_ARCH-static --version | head -n 1 | cut -d' ' -f3)

if ! which qemu-$QEMU_ARCH-static >/dev/null 2>&1; then
	echo -e "\e[35m"
	echo "Your qemu-$QEMU_ARCH-static(qemu-user-static) is missing"
	echo "Please install it:"
	echo "sudo apt-get install qemu-user-static"
	echo -e "\e[0m"
	exit 1
fi

if [ ${QEMU_VERSION%%.*} -lt 5 ]; then
	echo -e "\e[35m"
	echo "Your qemu-$QEMU_ARCH-static is too old: $QEMU_VERSION"
	echo "Please upgrade it:"
	if [ "$(uname -m)" = x86_64 ]; then
		echo "sudo update-binfmts --unimport qemu-$QEMU_ARCH 2>/dev/null"
		echo "sudo update-binfmts --disable qemu-$QEMU_ARCH 2>/dev/null"
		echo "sudo rm -f /usr/bin/qemu-$QEMU_ARCH-static"
		echo "sudo cp $RK_DATA_DIR/qemu-$QEMU_ARCH-static /usr/bin/"
		echo "sudo update-binfmts --enable qemu-$QEMU_ARCH 2>/dev/null"
		echo "sudo update-binfmts --import qemu-$QEMU_ARCH 2>/dev/null"
	else
		echo "https://www.qemu.org/download/"
	fi
	echo -e "\e[0m"
	exit 1
fi
