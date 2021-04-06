/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Configuration settings for the ChargePoint PBC
 * Derivation of the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX6PBC_CONFIG_H
#define __IMX6PBC_CONFIG_H

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#include "mx6_common.h"
#include "version.h"
#include "imx_env.h"

#define CONFIG_SYS_GENERIC_BOARD
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
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME			"eth0"
#else
#define CONFIG_ETHPRIME			"FEC"
#endif
#define CONFIG_FEC_MXC_PHYADDR		1
#endif

#define CONFIG_SPLASH_SCREEN

/*
 * SPLASH SCREEN Configs
 */
#if defined(CONFIG_SPLASH_SCREEN) && defined(CONFIG_MXC_EPDC)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_CMD_BMP
	#define CONFIG_LCD
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
		"earlyprintk=serial ignore_loglevel debug " \
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
	"bootenvpart=0:1\0" \
	"bootenv=uboot.env\0" \
	"bootfile=fitImage\0" \
	"bootparta=0:2\0" \
	"bootpartb=0:3\0" \
	"importbootenv=echo Importing env from mmc ${bootenvpart} ...; " \
		"if ext4load mmc ${bootenvpart} " \
			"${loadaddr} ${bootenv}; then " \
				"env import -c ${loadaddr} ${filesize} " \
					"trybootpart bootpart bootlabel; " \
		"elif ext4load mmc ${bootenvpart} " \
			"${loadaddr} ${bootenv}-backup; then " \
				"env import -c ${loadaddr} ${filesize} " \
					"bootpart bootlabel; " \
				"env export -c ${loadaddr} " \
					"trybootpart bootpart bootlabel; " \
				"ext4write mmc ${bootenvpart} ${loadaddr} " \
					"/${bootenv} ${filesize}; " \
		"fi; " \
	"\0" \
	"mmctryboot=" \
		"if test -n $trybootpart; then " \
			"echo Try-boot ${bootfile} from " \
				"mmc ${trybootpart} ...; " \
			"setenv -f _trybootpart ${trybootpart}; " \
			"setenv -f trybootpart; " \
			"env export -c ${loadaddr} " \
				"trybootpart bootpart bootlabel; " \
			"ext4write mmc ${bootenvpart} ${loadaddr} " \
				"/${bootenv} ${filesize}; " \
			"part uuid mmc ${_trybootpart} bootuuid; " \
			"setenv bootargs console=${console} " \
				"root=PARTUUID=${bootuuid} rootwait rw; " \
			"echo Try-booting ${bootfile} from mmc " \
				"${_trybootpart} ...; " \
			"ext4load mmc ${_trybootpart} ${loadaddr} " \
				"${bootfile} && " \
					"bootm ${loadaddr}; " \
		"fi; " \
	"\0" \
	"mmcboot=setenv -f _bootpart ${bootparta}; " \
		"run importbootenv; " \
		"run mmctryboot; " \
		"if test -n $bootpart && test $bootpart != none; then " \
			"setenv -f _bootpart ${bootpart}; " \
		"fi; " \
		"echo Booting ${bootfile} from mmc ${_bootpart} ...; " \
		"part uuid mmc ${_bootpart} bootuuid; " \
		"setenv bootargs console=${console} " \
			"root=PARTUUID=${bootuuid} rootwait rw; " \
		"ext4load mmc ${_bootpart} ${loadaddr} ${bootfile} && " \
			"bootm ${loadaddr}; " \
		"if test $_bootpart = $bootparta; then " \
			"setenv -f _bootpart ${bootpartb}; " \
		"else " \
			"setenv -f _bootpart ${bootparta}; " \
		"fi; " \
		"echo Failover boot ${bootfile} from mmc ${_bootpart} ...; " \
		"part uuid mmc ${_bootpart} bootuuid; " \
		"setenv bootargs console=${console} " \
			"root=PARTUUID=${bootuuid} rootwait rw; " \
		"ext4load mmc ${_bootpart} ${loadaddr} ${bootfile} && " \
			"bootm ${loadaddr}; " \
	"\0"

#define CONFIG_BOOTCOMMAND \
	"run mmcboot"

#define CONFIG_BOOTARGS \
	"console=" CONSOLE_DEV ",115200 earlyprintk=serial ignore_loglevel"

#endif                         /* __IMX6PBC_CONFIG_H */
