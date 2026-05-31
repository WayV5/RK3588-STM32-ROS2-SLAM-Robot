#!/bin/bash -e

SCRIPTS_DIR="${SCRIPTS_DIR:-$(dirname "$(realpath "$0")")}"

if ! ping google.com -c 1 -W 1 &>/dev/null; then
	echo -e "\e[35m"
	echo "Your network is not able to access google.com"
	echo "Please setup a VPN to bypass the GFW."
	echo -e "\e[0m"
	exit 1
fi

if ! which zstd >/dev/null 2>&1; then
	echo -e "\e[35m"
	echo "Your zstd is missing"
	echo "Please install it:"
	echo "sudo apt-get install zstd"
	echo -e "\e[0m"
	exit 1
fi

PYTHON3_MIN_VER=$(python3 --version | cut -d'.' -f2)
if [ "${PYTHON3_MIN_VER:-0}" -lt 6 ]; then
	echo -e "\e[35m"
	echo "Your python3 is too old for yocto: $(python3 --version)"
	echo "Please update it:"
	"$SCRIPTS_DIR/install-python3.sh"
	echo -e "\e[0m"
	exit 1
fi

# The yocto's e2fsprogs doesn't support new features like
# metadata_csum_seed and orphan_file
if grep -wq metadata_csum_seed /etc/mke2fs.conf; then
	echo -e "\e[35m"
	echo "Your mke2fs is too new: $(mke2fs -V 2>&1 | head -n 1)"
	echo "Please downgrade it:"
	"$SCRIPTS_DIR/install-e2fsprogs.sh"
	echo -e "\e[0m"
	exit 1
fi
