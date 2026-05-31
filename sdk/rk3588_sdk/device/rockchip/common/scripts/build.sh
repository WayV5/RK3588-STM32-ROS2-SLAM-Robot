#!/bin/bash

if [ -z "$BASH_SOURCE" ]; then
	echo "Not in bash, switching to it..."
	case "${@:-shell}" in
		shell) ./build.sh shell ;;
		*)
			./build.sh $@
			bash
			;;
	esac
fi

usage()
{
	echo "Usage: $(basename $BASH_SOURCE) [OPTIONS]"
	echo "Available options:"

	run_build_hooks usage

	# Global options
	echo -e "cleanall                          \tcleanup"
	echo -e "clean[:module[:module]]...        \tcleanup modules"
	echo "    available modules:"
	grep -wl clean_hook "$SCRIPTS_DIR"/mk-*.sh | \
		sed "s/^.*mk-\(.*\).sh/\t\1/"
	echo -e "post-rootfs <rootfs dir>          \ttrigger post-rootfs hook scripts"
	echo -e "help                              \tusage"
	echo ""
	echo "Default option is 'allsave'."

	rm -f "$INITIAL_ENV"
	exit 0
}

err_handler()
{
	ret=${1:-$?}
	[ "$ret" -eq 0 ] && return

	echo "ERROR: Running $BASH_SOURCE - ${2:-${FUNCNAME[1]}} failed!"
	echo "ERROR: exit code $ret from line ${BASH_LINENO[0]}:"
	echo "    ${3:-$BASH_COMMAND}"
	echo "ERROR: call stack:"
	for i in $(seq 1 $((${#FUNCNAME[@]} - 1))); do
		SOURCE="${BASH_SOURCE[$i]}"
		LINE=${BASH_LINENO[$(( $i - 1 ))]}
		echo "    $(basename "$SOURCE"): ${FUNCNAME[$i]}($LINE)"
	done
	exit $ret
}

# Export global functions
set -a

finish_build()
{
	echo -e "\e[35mRunning $(basename "${BASH_SOURCE[1]}") - ${@:-${FUNCNAME[1]}} succeeded.\e[0m"
	cd "$SDK_DIR"
}

load_config()
{
	[ -r "$RK_CONFIG" ] || return 0

	for var in $@; do
		export $(grep "^$var=" "$RK_CONFIG" | \
			tr -d '"' || true) &>/dev/null
	done
}

check_config()
{
	unset missing
	for var in $@; do
		eval [ \$$var ] && continue

		missing="$missing $var"
	done

	[ -z "$missing" ] && return 0

	echo "Skipping $(basename "${BASH_SOURCE[1]}") - ${FUNCNAME[1]} for missing configs: $missing."
	return 1
}

kernel_version_real()
{
	[ -d kernel ] || return 0

	VERSION_KEYS="VERSION PATCHLEVEL"
	VERSION=""

	for k in $VERSION_KEYS; do
		v=$(grep "^$k = " kernel/Makefile | cut -d' ' -f3)
		VERSION=${VERSION:+${VERSION}.}$v
	done
	echo $VERSION
}

kernel_version()
{
	[ -d kernel ] || return 0

	KERNEL_DIR="$(basename "$(realpath kernel)")"
	case "$KERNEL_DIR" in
		kernel-*)
			echo ${KERNEL_DIR#kernel-}
			return 0
			;;
	esac

	kernel_version_real
}

start_log()
{
	LOG_FILE="$RK_LOG_DIR/${2:-$1_$(date +%F_%H-%M-%S)}.log"
	ln -rsf "$LOG_FILE" "$RK_LOG_DIR/$1.log"
	echo "# $(date +"%F %T")" >> "$LOG_FILE"
	echo "$LOG_FILE"
}

get_toolchain()
{
	TOOLCHAIN_ARCH="${1/arm64/aarch64}"

	MACHINE=$(uname -m)
	if [ "$MACHINE" = x86_64 ]; then
		TOOLCHAIN_VENDOR="${2:-none}"
		TOOLCHAIN_OS="${3:-linux}"

		# RV1126 uses custom toolchain
		if [ "$RK_CHIP_FAMILY" = "rv1126_rv1109" ]; then
			TOOLCHAIN_VENDOR=rockchip
		fi

		TOOLCHAIN_DIR="$(realpath \
			$SDK_DIR/prebuilts/gcc/*/$TOOLCHAIN_ARCH)"
		GCC="$(find "$TOOLCHAIN_DIR"/*/bin -name "*gcc" 2>/dev/null | \
			grep -m 1 "$TOOLCHAIN_VENDOR-$TOOLCHAIN_OS-[^-]*-gcc")"
		if [ ! -x "$GCC" ]; then
			echo "No prebuilt GCC toolchain!"
			exit 1
		fi
	elif [ "$TOLLCHAIN_ARCH" = aarch64 -a "$MACHINE" != aarch64 ]; then
		GCC=aarch64-linux-gnu-gcc
	elif [ "$TOLLCHAIN_ARCH" = arm -a "$MACHINE" != armv7l ]; then
		GCC=arm-linux-gnueabihf-gcc
	else
		GCC=gcc
	fi

	echo "${GCC%gcc}"
}

# For developing shell only

rroot()
{
	cd "$SDK_DIR"
}

rout()
{
	cd "$RK_OUTDIR"
}

rcommon()
{
	cd "$COMMON_DIR"
}

rscript()
{
	cd "$SCRIPTS_DIR"
}

rchip()
{
	cd "$(realpath "$CHIP_DIR")"
}

set +a
# End of global functions

run_hooks()
{
	DIR="$1"
	shift

	for dir in "$CHIP_DIR/$(basename "$DIR")/" "$DIR"; do
		[ -d "$dir" ] || continue

		for hook in $(find "$dir" -maxdepth 1 -name "*.sh" | sort); do
			"$hook" $@ && continue
			HOOK_RET=$?
			err_handler $HOOK_RET "${FUNCNAME[0]} $*" "$hook $*"
			exit $HOOK_RET
		done
	done
}

run_build_hooks()
{
	# Don't log these hooks
	case "$1" in
		init | pre-build | make-* | usage | support-cmds)
			run_hooks "$RK_BUILD_HOOK_DIR" $@ || true
			return 0
			;;
	esac

	LOG_FILE="$(start_log "$1")"

	echo -e "# run hook: $@\n" >> "$LOG_FILE"
	run_hooks "$RK_BUILD_HOOK_DIR" $@ 2>&1 | tee -a "$LOG_FILE"
	HOOK_RET=${PIPESTATUS[0]}
	if [ $HOOK_RET -ne 0 ]; then
		err_handler $HOOK_RET "${FUNCNAME[0]} $*" "$@"
		exit $HOOK_RET
	fi
}

run_post_hooks()
{
	LOG_FILE="$(start_log post-rootfs)"

	echo -e "# run hook: $@\n" >> "$LOG_FILE"
	run_hooks "$RK_POST_HOOK_DIR" $@ 2>&1 | tee -a "$LOG_FILE"
	HOOK_RET=${PIPESTATUS[0]}
	if [ $HOOK_RET -ne 0 ]; then
		err_handler $HOOK_RET "${FUNCNAME[0]} $*" "$@"
		exit $HOOK_RET
	fi
}

option_check()
{
	CMDS="$1"
	shift

	for opt in $@; do
		for cmd in $CMDS; do
			# NOTE: There might be patterns in commands
			echo "${opt%%:*}" | grep -q "^$cmd$" || continue
			return 0
		done
	done

	return 1
}

main()
{
	[ -z "$DEBUG" ] || set -x

	trap 'err_handler' ERR
	set -eE

	# Save intial envionments
	unset INITIAL_SESSION
	INITIAL_ENV=$(mktemp -u)
	if [ -z "$RK_SESSION" ]; then
		INITIAL_SESSION=1
		env > "$INITIAL_ENV"
	fi

	export LC_ALL=C

	export SCRIPTS_DIR="$(dirname "$(realpath "$BASH_SOURCE")")"
	export COMMON_DIR="$(realpath "$SCRIPTS_DIR/..")"
	export SDK_DIR="$(realpath "$COMMON_DIR/../../..")"
	export DEVICE_DIR="$SDK_DIR/device/rockchip"
	export CHIPS_DIR="$DEVICE_DIR/.chips"
	export CHIP_DIR="$DEVICE_DIR/.chip"

	export RK_DATA_DIR="$COMMON_DIR/data"
	export RK_TOOL_DIR="$COMMON_DIR/tools"
	export RK_IMAGE_DIR="$COMMON_DIR/images"
	export RK_KBUILD_DIR="$COMMON_DIR/linux-kbuild"
	export RK_CONFIG_IN="$COMMON_DIR/configs/Config.in"

	export RK_BUILD_HOOK_DIR="$COMMON_DIR/build-hooks"
	export BUILD_HELPER="$RK_BUILD_HOOK_DIR/build-helper"
	export RK_POST_HOOK_DIR="$COMMON_DIR/post-hooks"
	export POST_HELPER="$RK_POST_HOOK_DIR/post-helper"

	export PARTITION_HELPER="$SCRIPTS_DIR/partition-helper"

	export RK_SESSION="${RK_SESSION:-$(date +%F_%H-%M-%S)}"

	export RK_OUTDIR="$SDK_DIR/output"
	export RK_SESSION_DIR="$RK_OUTDIR/sessions"
	export RK_LOG_BASE_DIR="$RK_OUTDIR/log"
	export RK_LOG_DIR="$RK_SESSION_DIR/$RK_SESSION"
	export RK_INITIAL_ENV="$RK_LOG_DIR/initial.env"
	export RK_CUSTOM_ENV="$RK_LOG_DIR/custom.env"
	export RK_FINAL_ENV="$RK_LOG_DIR/final.env"
	export RK_ROCKDEV_DIR="$SDK_DIR/rockdev"
	export RK_FIRMWARE_DIR="$RK_OUTDIR/firmware"
	export RK_SECURITY_FIRMWARE_DIR="$RK_OUTDIR/security-firmware"
	export RK_CONFIG="$RK_OUTDIR/.config"
	export RK_DEFCONFIG_LINK="$RK_OUTDIR/defconfig"

	# For Makefile
	case "$@" in
		make-targets)
			# Chip targets
			ls "$CHIPS_DIR"
			;&
		make-usage)
			run_build_hooks "$@"
			rm -f "$INITIAL_ENV"
			exit 0 ;;
	esac

	# Log SDK information
	MANIFEST="$SDK_DIR/.repo/manifest.xml"
	if [ -e "$MANIFEST" ]; then
		if [ ! -L "$MANIFEST" ]; then
			MANIFEST="$SDK_DIR/.repo/manifests/$(grep -o "[^\"]*\.xml" "$MANIFEST")"
		fi
		TAG="$(grep -o "linux-.*-gen-rkr[^.\"]*" "$MANIFEST" | \
			head -n 1 || true)"
		MANIFEST="$(basename "$(realpath "$MANIFEST")")"
		echo
		echo -e "\e[35m############### Rockchip Linux SDK ###############\e[0m"
		echo
		echo -e "\e[35mManifest: $MANIFEST\e[0m"
		if [ "$TAG" ]; then
			echo -e "\e[35mVersion: $TAG\e[0m"
		fi
		echo
	fi

	# Prepare firmware dirs
	mkdir -p "$RK_FIRMWARE_DIR" "$RK_SECURITY_FIRMWARE_DIR"

	cd "$SDK_DIR"
	[ -f README.md ] || ln -rsf "$COMMON_DIR/README.md" .
	[ -d common ] || ln -rsf "$COMMON_DIR" .

	# TODO: Remove it in the repo manifest.xml
	rm -f envsetup.sh

	OPTIONS=${@:-allsave}

	# Special handle for chip and defconfig
	# e.g. ./build.sh rk3588:rockchip_defconfig
	for opt in $OPTIONS; do
		if [ -d "$CHIPS_DIR/${opt%%:*}" ]; then
			OPTIONS=$(echo "$OPTIONS" | xargs -n 1 | \
				sed "s/^$opt$/chip:$opt/" | xargs)
		elif echo "$opt" | grep -q "^[0-9a-z_]*_defconfig$"; then
			OPTIONS=$(echo "$OPTIONS" | xargs -n 1 | \
				sed "s/^$opt$/defconfig:$opt/" | xargs)
		fi
	done

	# Options checking
	CMDS="$(run_build_hooks support-cmds all | xargs)"
	for opt in $OPTIONS; do
		case "$opt" in
			help | h | -h | --help | usage | \?) usage ;;
			clean:*)
				# Check cleanup modules
				for m in $(echo ${opt#clean:} | tr ':' ' '); do
					grep -wq clean_hook \
						"$SCRIPTS_DIR/mk-$m.sh" \
						2>/dev/null || usage
				done
				;&
			shell | cleanall)
				# Check single options
				if [ "$opt" = "$OPTIONS" ]; then
					break
				fi

				echo "ERROR: $opt cannot combine with other options!"
				;;
			post-rootfs)
				if [ "$opt" = "$1" -a -d "$2" ]; then
					# Hide other args from build stages
					OPTIONS=$opt
					break
				fi

				echo "ERROR: $opt should be the first option followed by rootfs dir!"
				;;
			*)
				# Make sure that all options are handled
				if option_check "$CMDS" $opt; then
					continue
				fi

				echo "ERROR: Unhandled option: $opt"
				;;
		esac

		usage
	done

	# Prepare log dirs
	if [ ! -d "$RK_LOG_DIR" ]; then
		rm -rf "$RK_LOG_BASE_DIR" "$RK_LOG_DIR" "$RK_SESSION_DIR/latest"
		mkdir -p "$RK_LOG_DIR"
		ln -rsf "$RK_SESSION_DIR" "$RK_LOG_BASE_DIR"
		ln -rsf "$RK_LOG_DIR" "$RK_SESSION_DIR/latest"
		echo -e "\e[33mLog saved at $RK_LOG_DIR\e[0m"
		echo
	fi

	# Drop old logs
	cd "$RK_LOG_BASE_DIR"
	rm -rf $(ls -t | sed '1,10d')
	cd "$SDK_DIR"

	# Save initial envionments
	if [ "$INITIAL_SESSION" ]; then
		rm -f "$RK_INITIAL_ENV"
		mv "$INITIAL_ENV" "$RK_INITIAL_ENV"
		ln -rsf "$RK_INITIAL_ENV" "$RK_OUTDIR/"
	fi

	# Init stage (preparing SDK configs, etc.)
	run_build_hooks init $OPTIONS
	rm -f "$RK_OUTDIR/.tmpconfig*"

	# No need to go further
	CMDS="$(run_build_hooks support-cmds pre-build build \
		post-build | xargs) cleanall clean post-rootfs"
	option_check "$CMDS" $OPTIONS || return 0

	# Force exporting config environments
	set -a

	# Load config environments
	source "$RK_CONFIG"
	cp "$RK_CONFIG" "$RK_LOG_DIR"

	if [ -z "$INITIAL_SESSION" ]; then
		# Inherit session environments
		sed -n 's/^\(RK_.*=\)\(.*\)/\1"\2"/p' "$RK_FINAL_ENV" > \
			"$INITIAL_ENV"
		source "$INITIAL_ENV"
		rm -f "$INITIAL_ENV"
	else
		# Detect and save custom environments

		# Find custom environments
		rm -f "$RK_CUSTOM_ENV"
		for cfg in $(grep "^RK_" "$RK_INITIAL_ENV" || true); do
			env | grep -q "^${cfg//\"/}$" || \
				echo "$cfg" >> "$RK_CUSTOM_ENV"
		done

		# Allow custom environments overriding
		if [ -e "$RK_CUSTOM_ENV" ]; then
			ln -rsf "$RK_CUSTOM_ENV" "$RK_OUTDIR/"

			echo -e "\e[31mWARN: Found custom environments: \e[0m"
			cat "$RK_CUSTOM_ENV"

			echo -e "\e[31mAssuming that is expected, please clear them if otherwise.\e[0m"
			read -t 10 -p "Press enter to continue."
			source "$RK_CUSTOM_ENV"

			if grep -q "^RK_KERNEL_VERSION=" "$RK_CUSTOM_ENV"; then
				echo -e "\e[31mCustom RK_KERNEL_VERSION ignored!\e[0m"
				load_config RK_KERNEL_VERSION
			fi

			if grep -q "^RK_ROOTFS_SYSTEM=" "$RK_CUSTOM_ENV"; then
				echo -e "\e[31mCustom RK_ROOTFS_SYSTEM ignored!\e[0m"
				load_config RK_ROOTFS_SYSTEM
			fi
		fi
	fi

	source "$PARTITION_HELPER"
	rk_partition_init

	set +a

	export PYTHON3=/usr/bin/python3
	export RK_KERNEL_VERSION_REAL=$(kernel_version_real)

	# Handle special commands
	case "$OPTIONS" in
		cleanall)
			run_build_hooks clean
			rm -rf "$RK_OUTDIR" "$SDK_DIR/rockdev"
			finish_build cleanall
			exit 0 ;;
		clean:*)
			MODULES="$(echo ${OPTIONS#clean:} | tr ':' ' ')"
			for m in $MODULES; do
				"$SCRIPTS_DIR/mk-$m.sh" clean
			done
			finish_build clean - $MODULES
			exit 0 ;;
		post-rootfs)
			shift
			run_post_hooks $@
			finish_build post-rootfs
			exit 0 ;;
	esac

	# Save final environments
	rm -f "$RK_FINAL_ENV"
	env > "$RK_FINAL_ENV"
	ln -rsf "$RK_FINAL_ENV" "$RK_OUTDIR/"

	# Log configs
	echo
	echo "=========================================="
	echo "          Final configs"
	echo "=========================================="
	env | grep -E "^RK_.*=.+" | grep -vE "PARTITION_[0-9]" | \
		grep -vE "=\"\"$|_DEFAULT=y" | \
		grep -vE "^RK_CONFIG|_BASE_CFG=|_LINK=|DIR=|_ENV=|_NAME=" | sort
	echo

	# Pre-build stage (submodule configuring, etc.)
	run_build_hooks pre-build $OPTIONS

	# No need to go further
	CMDS="$(run_build_hooks support-cmds build post-build | xargs)"
	option_check "$CMDS" $OPTIONS || return 0

	# Build stage (building, etc.)
	run_build_hooks build $OPTIONS

	# No need to go further
	CMDS="$(run_build_hooks support-cmds post-build | xargs)"
	option_check "$CMDS" $OPTIONS || return 0

	# Post-build stage (firmware packing, etc.)
	run_build_hooks post-build $OPTIONS
}

if [ "$0" != "$BASH_SOURCE" ]; then
	# Sourced, executing it directly
	"$BASH_SOURCE" ${@:-shell}
elif [ "$0" == "$BASH_SOURCE" ]; then
	# Executed directly
	main $@
fi
