#!/bin/bash -e

# The rtos needs scons support
if ! scons -v >/dev/null; then
	echo -e "\e[35m"
	echo "Your scons is missing"
	echo "Please install it:"
	echo "sudo apt-get install scons"
	echo -e "\e[0m"
	exit 1
fi
