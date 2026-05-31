#!/bin/bash -e

MISC_IMG="$(realpath "${1:-misc.img}")"
[ -z "$1" ] || shift
COMMAND="$1"
[ -z "$1" ] || shift
ARGS="$@"

rm -rf "$MISC_IMG"
# The old windows tools don't accept misc > 64K
truncate -s 48k "$MISC_IMG"

case "$COMMAND" in
	"") echo -e "Generated blank misc image:\n$MISC_IMG" ;;
	recovery)
		echo -n "boot-recovery" | \
			dd of="$MISC_IMG" bs=1 seek=$((16*1024)) conv=notrunc
		echo -e -n "$COMMAND\n$ARGS" | \
			dd of="$MISC_IMG" bs=1 seek=$((16*1024+64)) conv=notrunc
		echo -e "Generated misc image for \"$COMMAND $ARGS\":\n$MISC_IMG"
		;;
	*) echo "Unsupported command: $COMMAND $ARGS!" ;;
esac
