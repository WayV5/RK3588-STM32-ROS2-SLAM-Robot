#!/bin/bash -e

SCRIPTS_DIR="${SCRIPTS_DIR:-$(dirname "$(realpath "$0")")}"
SDK_DIR="${SDK_DIR:-$SCRIPTS_DIR/../../../..}"
RK_DATA_DIR="${RK_DATA_DIR:-$SCRIPTS_DIR/../data}"

cd "$SDK_DIR"

"$SCRIPTS_DIR/check-grow-align.sh"

if [ -r "kernel/.config" ]; then
	EXT4_CONFIGS=$(export | grep -oE "\<RK_.*=\"ext4\"$" || true)

	if [ "$EXT4_CONFIGS" ] && \
		! grep -q "CONFIG_EXT4_FS=y" kernel/.config; then
		echo -e "\e[35m"
		echo "Your kernel doesn't support ext4 filesystem"
		echo "Please enable CONFIG_EXT4_FS for:"
		echo "$EXT4_CONFIGS"
		echo -e "\e[0m"
		exit 1
	fi
fi

if ! kernel/scripts/mkbootimg &>/dev/null; then
	echo -e "\e[35m"
	echo "Your python3 is too old for kernel: $(python3 --version)"
	echo "Please update it:"
	"$SCRIPTS_DIR/install-python3.sh"
	echo -e "\e[0m"
	exit 1
fi

if ! lz4 -h 2>&1 | grep -q favor-decSpeed; then
	echo -e "\e[35m"
	echo "Your lz4 is too old for kernel: $(lz4 --version)"
	echo "Please update it:"
	echo "git clone https://github.com/lz4/lz4.git --depth 1 -b v1.9.4"
	echo "cd lz4"
	echo "sudo make install -j8"
	echo -e "\e[0m"
	exit 1
fi

"$SCRIPTS_DIR/check-header.sh" openssl openssl/ssl.h libssl-dev
"$SCRIPTS_DIR/check-header.sh" gmp gmp.h libgmp-dev
"$SCRIPTS_DIR/check-header.sh" mpc mpc.h libmpc-dev
