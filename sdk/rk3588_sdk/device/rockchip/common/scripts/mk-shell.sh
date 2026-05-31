#!/bin/bash -e

# Hooks

usage_hook()
{
	echo -e "shell                             \tsetup a shell for developing"
}

PRE_BUILD_CMDS="shell"
pre_build_hook()
{
	echo -e "\e[35mDoing this is dangerous and for developing only.\e[0m"
	# No error handling in develop shell.
	set +e; trap ERR
	/bin/bash
	echo -e "\e[35mExit from $BASH_SOURCE shell.\e[0m"
}

source "${BUILD_HELPER:-$(dirname "$(realpath "$0")")/../build-hooks/build-helper}"

pre_build_hook
