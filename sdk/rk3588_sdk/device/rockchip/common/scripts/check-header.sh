#!/bin/bash -e

SCRIPTS_DIR="${SCRIPTS_DIR:-$(dirname "$(realpath "$0")")}"
RK_DATA_DIR="${RK_DATA_DIR:-$SCRIPTS_DIR/../data}"

PACKAGE="$1"
HEADER="$2"
APT_PACKAGE="$3"

if echo | gcc -E -include "$HEADER" - &>/dev/null; then
	exit 0
fi

echo -e "\e[35m"
echo "Your $PACKAGE headers are missing"
echo "Please install it:"
echo "sudo apt-get install $APT_PACKAGE"
echo -e "\e[0m"
exit 1
