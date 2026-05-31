#

if RK_KERNEL_CFG != ""

menu "Boot"

config RK_BOOT_IMG
	string
	default "zboot.img" if RK_BOOT_COMPRESSED
	default "boot.img"

config RK_BOOT_COMPRESSED
	bool "compressed boot image (zboot)"
	default y if RK_CHIP_FAMILY = "rv1126_rv1109" || \
		RK_CHIP_FAMILY = "px30" || RK_CHIP_FAMILY = "px3se" || \
		RK_CHIP_FAMILY = "rk3036" || RK_CHIP_FAMILY = "rk3128h" || \
		RK_CHIP_FAMILY = "rk312x" || RK_CHIP_FAMILY = "rk3229" || \
		RK_CHIP_FAMILY = "rk3288" || RK_CHIP_FAMILY = "rk3308" || \
		RK_CHIP_FAMILY = "rk3326" || RK_CHIP_FAMILY = "rk3358"

config RK_BOOT_FIT_ITS
	string "its script for FIT boot image"
	depends on RK_USE_FIT_IMG
	default "boot.its" if RK_CHIP_FAMILY = "rv1126_rv1109"
	default "zboot.its" if RK_BOOT_COMPRESSED
	default "boot.its"

endmenu # Boot

endif
