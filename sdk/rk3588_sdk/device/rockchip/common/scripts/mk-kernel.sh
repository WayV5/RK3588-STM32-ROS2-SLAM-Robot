#!/bin/bash -e

KERNELS=$(ls | grep kernel- || true)

update_kernel()
{
	# Fallback to current kernel
	RK_KERNEL_VERSION=${RK_KERNEL_VERSION:-$(kernel_version)}

	# Fallback to 5.10 kernel
	RK_KERNEL_VERSION=${RK_KERNEL_VERSION:-5.10}

	# Update .config
	KERNEL_CONFIG="RK_KERNEL_VERSION=\"$RK_KERNEL_VERSION\""
	if ! grep -q "^$KERNEL_CONFIG$" "$RK_CONFIG"; then
		sed -i "s/^RK_KERNEL_VERSION=.*/$KERNEL_CONFIG/" "$RK_CONFIG"
		"$SCRIPTS_DIR/mk-config.sh" olddefconfig &>/dev/null
	fi

	[ "$(kernel_version)" != "$RK_KERNEL_VERSION" ] || return 0

	# Update kernel
	KERNEL_DIR=kernel-$RK_KERNEL_VERSION
	echo "switching to $KERNEL_DIR"
	if [ ! -d "$KERNEL_DIR" ]; then
		echo "$KERNEL_DIR not exist!"
		exit 1
	fi

	rm -rf kernel
	ln -rsf $KERNEL_DIR kernel
}

do_build()
{
	if [ "$DRY_RUN" ]; then
		echo -e "\e[35mCommands of building $1:\e[0m"
	else
		echo "=========================================="
		echo "          Start building $1"
		echo "=========================================="
	fi

	check_config RK_KERNEL_DTS_NAME RK_KERNEL_CFG RK_BOOT_IMG || return 0

	run_command $KMAKE $RK_KERNEL_CFG $RK_KERNEL_CFG_FRAGMENTS

	if [ -z "$DRY_RUN" ]; then
		"$SCRIPTS_DIR/check-kernel.sh"
	fi

	case "$1" in
		kernel-config)
			KERNEL_CONFIG_DIR="kernel/arch/$RK_KERNEL_ARCH/configs"
			run_command $KMAKE menuconfig
			run_command $KMAKE savedefconfig
			run_command mv kernel/defconfig \
				"$KERNEL_CONFIG_DIR/$RK_KERNEL_CFG"
			;;
		kernel*)
			#run_command $KMAKE "$RK_KERNEL_DTS_NAME.img"
			cd $SDK_DIR/kernel/ && ./make.sh --skip-config board="$RK_TARGET_BOARD" \
			arch="$RK_KERNEL_ARCH" defconfig="$RK_KERNEL_CFG $RK_KERNEL_CFG_FRAGMENTS" addon_arg="CROSS_COMPILE=$RK_KERNEL_TOOLCHAIN" dts="$RK_KERNEL_DTS_NAME" jobs="$RK_BUILD_JOBS" \
			&& cd -


			# The FIT image for initrd would be packed in rootfs stage
			if [ -n "$RK_BOOT_FIT_ITS" ]; then
				if [ -z "$RK_ROOTFS_INITRD" ]; then
					run_command \
						"$SCRIPTS_DIR/mk-fitimage.sh" \
						"kernel/$RK_BOOT_IMG" \
						"$RK_BOOT_FIT_ITS" \
						"$RK_KERNEL_IMG"
				fi
			fi

			if [ "$RK_WIFIBT_CHIP" ] && [ -r "$RK_KERNEL_DTB" ] && \
				! grep -wq wireless-bluetooth "$RK_KERNEL_DTB"; then
				echo -e "\e[35m"
				echo "Missing wireless-bluetooth in $RK_KERNEL_DTS!"
				echo -e "\e[0m"
			fi
			;;
		modules) run_command $KMAKE modules ;;
	esac
}

# Hooks

usage_hook()
{
	for k in $KERNELS; do
		echo -e "$k[:cmds]               \tbuild kernel ${k#kernel-}"
	done

	echo -e "kernel[:cmds]                    \tbuild kernel"
	echo -e "modules[:cmds]                   \tbuild kernel modules"
	echo -e "linux-headers[:cmds]             \tbuild linux-headers"
	echo -e "kernel-config[:cmds]             \tmodify kernel defconfig"
	echo -e "kernel-make[:<arg1>:<arg2>]      \trun kernel make (alias kmake)"
}

clean_hook()
{
	[ ! -d kernel ] || make -C kernel distclean
	rm -f "$RK_OUTDIR/linux-headers.tar"
}

INIT_CMDS="default $KERNELS"
init_hook()
{
	load_config RK_KERNEL_CFG
	check_config RK_KERNEL_CFG &>/dev/null || return 0

	# Priority: cmdline > custom env > .config > current kernel/ symlink
	if echo $1 | grep -q "^kernel-"; then
		export RK_KERNEL_VERSION=${1#kernel-}
		echo "Using kernel version($RK_KERNEL_VERSION) from cmdline"
	elif [ "$RK_KERNEL_VERSION" ]; then
		export RK_KERNEL_VERSION=${RK_KERNEL_VERSION//\"/}
		echo "Using kernel version($RK_KERNEL_VERSION) from environment"
	else
		load_config RK_KERNEL_VERSION
	fi

	update_kernel
}

PRE_BUILD_CMDS="kernel-config kernel-make kmake"
pre_build_hook()
{
	check_config RK_KERNEL_CFG || return 0
	source "$SCRIPTS_DIR/kernel-helper"

	echo "Toolchain for kernel:"
	echo "${RK_KERNEL_TOOLCHAIN:-gcc}"
	echo

	case "$1" in
		kernel-make | kmake)
			shift
			[ "$1" != cmds ] || shift

			if [ "$DRY_RUN" ]; then
				echo -e "\e[35mCommands of building ${@:-stuff}:\e[0m"
			else
				echo "=========================================="
				echo "          Start building $@"
				echo "=========================================="
			fi

			if [ ! -r kernel/.config ]; then
				run_command $KMAKE $RK_KERNEL_CFG \
					$RK_KERNEL_CFG_FRAGMENTS
			fi
			run_command $KMAKE $@
			;;
		kernel-config)
			do_build $@
			;;
	esac

	if [ -z "$DRY_RUN" ]; then
		finish_build $@
	fi
}

pre_build_hook_dry()
{
	DRY_RUN=1 pre_build_hook $@
}

BUILD_CMDS="$KERNELS kernel modules"
build_hook()
{
	check_config RK_KERNEL_DTS_NAME RK_KERNEL_CFG RK_BOOT_IMG || return 0
	source "$SCRIPTS_DIR/kernel-helper"

	echo "Toolchain for kernel:"
	echo "${RK_KERNEL_TOOLCHAIN:-gcc}"
	echo

	if echo $1 | grep -q "^kernel-"; then
		if [ "$RK_KERNEL_VERSION" != "${1#kernel-}" ]; then
			echo -ne "\e[35m"
			echo "Kernel version overrided: " \
				"$RK_KERNEL_VERSION -> ${1#kernel-}"
			echo -ne "\e[0m"
		fi
	fi

	do_build $@

	if [ "$DRY_RUN" ]; then
		return 0
	fi

	if echo $1 | grep -q "^kernel"; then
		ln -rsf "kernel/$RK_BOOT_IMG" "$RK_FIRMWARE_DIR/boot.img"
		"$SCRIPTS_DIR/check-power-domain.sh"
	fi

	finish_build build_$1
}

build_hook_dry()
{
	DRY_RUN=1 build_hook $@
}

POST_BUILD_CMDS="linux-headers"
post_build_hook()
{
	check_config RK_KERNEL_DTS_NAME RK_KERNEL_CFG RK_BOOT_IMG || return 0
	source "$SCRIPTS_DIR/kernel-helper"

	[ "$1" = "linux-headers" ] || return 0
	shift

	[ "$1" != cmds ] || shift
	OUTPUT_FILE="${2:-"$RK_OUTDIR"}/linux-headers.tar"
	mkdir -p "$(dirname "OUTPUT_DIR")"

	HEADER_FILES_SCRIPT=$(mktemp)

	if [ "$DRY_RUN" ]; then
		echo -e "\e[35mCommands of building linux-headers:\e[0m"
	else
		echo "Saving linux-headers to $OUTPUT_FILE"
	fi

	run_command $KMAKE $RK_KERNEL_CFG $RK_KERNEL_CFG_FRAGMENTS
	run_command $KMAKE modules_prepare

	cat << EOF > "$HEADER_FILES_SCRIPT"
{
	# Based on kernel/scripts/package/builddeb
	find . arch/$RK_KERNEL_ARCH -maxdepth 1 -name Makefile\*
	find include -type f -o -type l
	find arch/$RK_KERNEL_ARCH -name module.lds -o -name Kbuild.platforms -o -name Platform
	find \$(find arch/$RK_KERNEL_ARCH -name include -o -name scripts -type d) -type f
	find arch/$RK_KERNEL_ARCH/include Module.symvers -type f
	echo .config
} | tar --no-recursion --ignore-failed-read -T - \
	-cf "$OUTPUT_FILE"
EOF

	run_command cd "$SDK_DIR/kernel"

	cat "$HEADER_FILES_SCRIPT"

	if [ -z "$DRY_RUN" ]; then
		. "$HEADER_FILES_SCRIPT"
	fi

	case "$RK_KERNEL_KBUILD_ARCH" in
		host) run_command tar -uf "$OUTPUT_FILE" scripts tools ;;
		*)
			run_command cd "$RK_KBUILD_DIR/$RK_KERNEL_KBUILD_ARCH"
			run_command cd "linux-kbuild-$RK_KERNEL_VERSION_REAL"
			run_command tar -uf "$OUTPUT_FILE" .
			;;
	esac

	run_command cd "$SDK_DIR"

	rm -f "$HEADER_FILES_SCRIPT"
}

post_build_hook_dry()
{
	DRY_RUN=1 post_build_hook $@
}

source "${BUILD_HELPER:-$(dirname "$(realpath "$0")")/../build-hooks/build-helper}"

case "${1:-kernel}" in
	kernel-config | kernel-make | kmake) pre_build_hook $@ ;;
	kernel* | modules)
		init_hook $@
		build_hook ${@:-kernel}
		;;
	linux-headers) post_build_hook $@ ;;
	*) usage ;;
esac
