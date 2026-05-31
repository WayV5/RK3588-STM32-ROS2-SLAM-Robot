#!/bin/bash -e

# mk-image.sh requires -d option to pack e2fs for non-root user
if ! mke2fs -h 2>&1 | grep -wq "\-d"; then
	echo -e "\e[35m"
	echo "Your mke2fs is too old: $(mke2fs -V 2>&1 | head -n 1)"
	echo "Please update it:"
	"$SCRIPTS_DIR/install-e2fsprogs.sh"
	echo -e "\e[0m"
	exit 1
fi
