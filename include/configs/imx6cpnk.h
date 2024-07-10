/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 * Copyright 2020 ChargePoint, Inc.
 *
 * Configuration settings for the ChargePoint CPNK
 * Derivation of the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX6CPNK_CONFIG_H
#define __IMX6CPNK_CONFIG_H

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#include "mx6_common.h"
#include "version.h"
#include "imx_env.h"

#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_SERIAL_TAG
#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

/* Size of image extraction should match bootm */
#define CONFIG_SYS_XIMG_LEN		CONFIG_SYS_BOOTM_LEN

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			(4UL * SZ_1G) /* 4GB DDR3L */

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH	0x10800000

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * SZ_1K)
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define FDT_SEQ_MACADDR_FROM_ENV

/* FIT */
#ifdef CONFIG_FIT
#define CONFIG_IMAGE_FORMAT_LEGACY
#endif

/* Serial */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV	"ttymxc0"

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	2

#define CONFIG_SYS_MMC_ENV_DEV		0  /* MMC0 */
#if defined(CONFIG_ENV_IS_IN_MMC)
#ifndef CONFIG_SYS_MMC_ENV_PART
#define CONFIG_SYS_MMC_ENV_PART		0  /* user partition */
#endif
#endif

#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

/* Re-using the machine type for a sabresd since it is close enough */
#define CONFIG_MACH_TYPE	MACH_TYPE_MX6Q_SABRESD

#ifdef CONFIG_SYS_USE_SPINOR
#define CONFIG_SF_DEFAULT_CS   0
#endif

/*
 * imx6 q/dl/solo pcie would be failed to work properly in kernel, if
 * the pcie module is iniialized/enumerated both in uboot and linux
 * kernel.
 * rootcause:imx6 q/dl/solo pcie don't have the reset mechanism.
 * it is only be RESET by the POR. So, the pcie module only be
 * initialized/enumerated once in one POR.
 * Set to use pcie in kernel defaultly, mask the pcie config here.
 * Remove the mask freely, if the uboot pcie functions, rather than
 * the kernel's, are required.
 */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(7, 12)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(3, 19)
#endif

#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_SPEED 20000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

/* I2C Configs */
#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000
#endif
#endif

/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#endif

/* OCOTP Configs */
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* PMIC */
#ifndef CONFIG_DM_PMIC
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08
#endif

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif
#define CONFIG_USBD_HS
#define CONFIG_FASTBOOT_USB_DEV 0

/* Network Configs */
#ifdef CONFIG_CMD_NET
#define CONFIG_MII
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS
#endif

/*
 * SPLASH SCREEN Configs
 */
#if defined(CONFIG_SPLASH_SCREEN) && defined(CONFIG_MXC_EPDC)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_CMD_BMP
	#undef LCD_TEST_PATTERN
	/* #define CONFIG_SPLASH_IS_IN_MMC			1 */
	#define LCD_BPP					LCD_MONOCHROME
	/* #define CONFIG_SPLASH_SCREEN_ALIGN		1 */

	#define CONFIG_WAVEFORM_BUF_SIZE		0x400000
#endif /* CONFIG_SPLASH_SCREEN && CONFIG_MXC_EPDC */


/*
 * Define the builtin software environment settings
 */
#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"reboot=h earlyprintk=serial ignore_loglevel debug " \
		"clk_ignore_unused root=/dev/ram " \
		"\0" \
	"bootcmd_mfg=run mfgtool_args; " \
		"echo \"Run fastboot ...\"; fastboot 0; "  \
		"\0" \

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"console=" CONSOLE_DEV "\0" \
	"video=mxcfb0:dev=ldb,1024x768M@60,bpp=32,if=RGB24\0" \
	"ota_triggered=0\0" \
	"ota_errors=0\0" \
	"compat_mmcdev=0\0" \
	"compat_mmcpart=5\0" \
	"compat_ota_engine=OTA_uImage.imx6q\0" \
	"compat_ota_filename=ota.tar.xz\0" \
	"compat_ota_mmcpart=3\0" \
	"compat_ota_device=/dev/mmcblk0p4\0" \
	"compat_image=/boot/zImage\0" \
	"compat_fdt_file=/boot/imx6qp-sabresd-cpnk.dtb\0" \
	"compat_fdt_addr=0x18000000\0" \
	"bootenvpart=0:b\0" \
	"bootenv=uboot.env\0" \
	"bootfile=fitImage\0" \
	"bootparta=0:c\0" \
	"bootpartb=0:d\0" \
	"importbootenv=echo Importing env from mmc ${bootenvpart} ...; " \
		"part uuid mmc ${bootenvpart} bootenvuuid; " \
		"if ext4load mmc ${bootenvpart} " \
			"${loadaddr} ${bootenv}; then " \
				"env import -c ${loadaddr} ${filesize} " \
					"display fitconfig " \
					"trybootpart bootpart bootlabel; " \
		"elif ext4load mmc ${bootenvpart} " \
			"${loadaddr} ${bootenv}-backup; then " \
				"env import -c ${loadaddr} ${filesize} " \
					"display fitconfig " \
					"bootpart bootlabel; " \
				"env export -c ${loadaddr} " \
					"display fitconfig " \
					"trybootpart bootpart bootlabel; " \
				"ext4write mmc ${bootenvpart} ${loadaddr} " \
					"/${bootenv} ${filesize}; " \
		"fi; " \
	"\0" \
	"mmctryboot=" \
		"if test -n ${trybootpart}; then " \
			"echo Try-boot ${bootfile} from " \
				"mmc ${trybootpart} ...; " \
			"setenv -f _trybootpart ${trybootpart}; " \
			"setenv -f trybootpart; " \
			"env export -c ${loadaddr} " \
				"display fitconfig " \
				"trybootpart bootpart bootlabel; " \
			"ext4write mmc ${bootenvpart} ${loadaddr} " \
				"/${bootenv} ${filesize}; " \
			"part uuid mmc ${_trybootpart} bootuuid; " \
			"setenv bootargs ${bootargs_secureboot} " \
				"console=${console} " \
				"video=${video} " \
				"bootenv=PARTUUID=${bootenvuuid} " \
				"root=PARTUUID=${bootuuid} rootwait rw " \
				"${bootargs_append}; " \
			"echo Try-booting ${bootfile} from mmc " \
				"${_trybootpart} ...; " \
			"ext4load mmc ${_trybootpart} ${loadaddr} " \
				"${bootfile} && " \
					"bootm ${bootmarg}; " \
		"fi; " \
	"\0" \
	"mmcboot=setenv -f _bootpart ${bootparta}; " \
		"run importbootenv; " \
		"extension scan; " \
		"extension fitconfig; " \
		"setenv -f bootmarg ${loadaddr}; " \
		"if test -n ${extension_fitconfig}; then " \
			"setenv -f bootmarg " \
				"${loadaddr}${extension_fitconfig}; " \
		"fi; " \
		"run mmctryboot; " \
		"if test -n ${bootpart} && test ${bootpart} != none; then " \
			"setenv -f _bootpart ${bootpart}; " \
		"fi; " \
		"echo Booting ${bootfile} from mmc ${_bootpart} ...; " \
		"part uuid mmc ${_bootpart} bootuuid; " \
		"setenv bootargs ${bootargs_secureboot} " \
			"console=${console} video=${video} " \
			"bootenv=PARTUUID=${bootenvuuid} " \
			"root=PARTUUID=${bootuuid} rootwait rw " \
			"${bootargs_append}; " \
		"ext4load mmc ${_bootpart} ${loadaddr} ${bootfile} && " \
			"bootm ${bootmarg}; " \
		"if test ${_bootpart} = ${bootparta}; then " \
			"setenv -f _bootpart ${bootpartb}; " \
		"else " \
			"setenv -f _bootpart ${bootparta}; " \
		"fi; " \
		"echo Failover boot ${bootfile} from mmc ${_bootpart} ...; " \
		"part uuid mmc ${_bootpart} bootuuid; " \
		"setenv bootargs ${bootargs_secureboot} " \
			"console=${console} video=${video} " \
			"bootenv=PARTUUID=${bootenvuuid} " \
			"root=PARTUUID=${bootuuid} rootwait rw " \
			"systemd.unit=rescue.target " \
			"${bootargs_append}; " \
		"ext4load mmc ${_bootpart} ${loadaddr} ${bootfile} && " \
			"bootm ${bootmarg}; " \
	"\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev 0 2; " \
	"mmc read ${loadaddr} 0x400 0x10; " \
	"env import -c ${loadaddr} 0x2000 " \
		"ota_filename ota_triggered ota_errors; " \
	"mmc dev 0; " \
	"if test -z \"${ota_filename}\"; then "\
		"setenv -f ota_filename ${compat_ota_filename}; " \
	"fi; " \
	"if test ${ota_triggered} -eq 1; then " \
		"if ext4load mmc ${compat_mmcdev}:${compat_ota_mmcpart} " \
			"${loadaddr} ${compat_ota_engine}; then " \
			"setenv bootargs console=${console},${baudrate} " \
				"video=${video} " \
				"ota_device=${compat_ota_device} " \
				"ota_filename=${ota_filename}; " \
			"bootm ${loadaddr}; " \
		"fi; " \
	"fi; " \
	"if ext4load mmc ${compat_mmcdev}:${compat_mmcpart} " \
		"${loadaddr} ${compat_image}; then " \
		"setenv bootargs console=${console},${baudrate} " \
			"video=${video} " \
			"root=/dev/mmcblk0p5 rootwait rw ${bootargs_append}; " \
		"if ext4load mmc ${compat_mmcdev}:${compat_mmcpart} " \
			"${compat_fdt_addr} ${compat_fdt_file}; then " \
			"bootz ${loadaddr} - ${compat_fdt_addr}; " \
		"else " \
			"bootz; " \
		"fi; " \
	"fi; " \
	"run mmcboot"

#define CONFIG_BOOTARGS \
	"console=" CONSOLE_DEV ",115200 earlyprintk=serial ignore_loglevel"

#endif                         /* __IMX6CPNK_CONFIG_H */
