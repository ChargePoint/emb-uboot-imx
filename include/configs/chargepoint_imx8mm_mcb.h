/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 NXP
 */

#ifndef __IMX8MM_MCB_H
#define __IMX8MM_MCB_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#define CONFIG_SPL_MAX_SIZE		(148 * 1024)
#define CONFIG_SYS_MONITOR_LEN		SZ_512K
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	(0x300 + CONFIG_SECONDARY_BOOT_SECTOR_OFFSET)
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SPL_STACK		0x920000
#define CONFIG_SPL_BSS_START_ADDR	0x910000
#define CONFIG_SPL_BSS_MAX_SIZE		SZ_8K	/* 8 KB */
#define CONFIG_SYS_SPL_MALLOC_START	0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE	SZ_512K	/* 512 KB */

/* malloc f used before GD_FLG_FULL_MALLOC_INIT set */
#define CONFIG_MALLOC_F_ADDR		0x912000
/* For RAW image gives a error info not panic */
#define CONFIG_SPL_ABORT_ON_RAW_IMAGE

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#if defined(CONFIG_IMX8M_LPDDR4) && defined(CONFIG_TARGET_CHARGEPOINT_IMX8MM_MCB)
#define CONFIG_POWER_PCA9450
#else
#define CONFIG_POWER_BD71837
#endif

#define CONFIG_SYS_I2C

#endif

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_REMAKE_ELF

#define CONSOLE_DEV	"ttymxc1"

#define CONFIG_MFG_ENV_SETTINGS \
	"console=" CONSOLE_DEV "\0" \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"reboot=h earlyprintk=serial ignore_loglevel debug " \
		"clk_ignore_unused root=/dev/ram " \
		"\0" \
	"bootcmd_mfg=run mfgtool_args; " \
		"echo \"Run fastboot ...\"; fastboot 0; "  \
		"\0" \


#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"bootenvpart=2:1\0" \
	"bootenv=uboot.env\0" \
	"bootfile=fitImage\0" \
	"bootparta=2:2\0" \
	"bootpartb=2:3\0" \
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
				"console=${console},${baudrate} " \
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
		"setenv -f bootmarg ${loadaddr}; " \
		"if test -n ${fitconfig}; then " \
			"setenv -f bootmarg ${loadaddr}#${fitconfig}; " \
		"fi; " \
		"run mmctryboot; " \
		"if test -n ${bootpart} && test ${bootpart} != none; then " \
			"setenv -f _bootpart ${bootpart}; " \
		"fi; " \
		"echo Booting ${bootfile} from mmc ${_bootpart} ...; " \
		"part uuid mmc ${_bootpart} bootuuid; " \
		"setenv bootargs ${bootargs_secureboot} console=${console} " \
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
		"setenv bootargs ${bootargs_secureboot} console=${console} " \
			"bootenv=PARTUUID=${bootenvuuid} " \
			"root=PARTUUID=${bootuuid} rootwait rw " \
			"systemd.unit=rescue.target " \
			"${bootargs_append}; " \
		"ext4load mmc ${_bootpart} ${loadaddr} ${bootfile} && " \
			"bootm ${bootmarg}; " \
	"\0"


#define CONFIG_BOOTCOMMAND \
	"run mmcboot"

#define CONFIG_BOOTARGS \
	"console=" CONSOLE_DEV ",115200 earlyprintk=serial ignore_loglevel"

/*
 * Link Definitions
 *
 * NOTE: loadaddr tracks the fastboot buffer for a fit image boot, the
 *       size and location of a buffer allows the relocation and extraction
 *       of the kernel to lower memory and the initrd to relocate to upper
 *       memory. This is a deviation from the imx8mm references that don't
 *       use FIT images.
 */
#define CONFIG_LOADADDR			0x71000000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_SYS_INIT_RAM_ADDR        0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE        0x200000
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_SYS_MMC_ENV_DEV		1   /* USDHC2 */
#define CONFIG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		SZ_32M

/* support for larger ramdisk */
#define CONFIG_SYS_BOOTM_LEN            (128 << 20)
#define CONFIG_SYS_XIMG_LEN             CONFIG_SYS_BOOTM_LEN

#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                      0x40000000
#define PHYS_SDRAM_SIZE			0x40000000 /* 1GB DDR */

#define CONFIG_SYS_MEMTEST_START	PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + (PHYS_SDRAM_SIZE >> 1))

#define CONFIG_MXC_UART_BASE		UART2_BASE_ADDR

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_IMX_BOOTAUX

/* USDHC */
#define CONFIG_FSL_USDHC

#define CONFIG_SYS_FSL_USDHC_NUM		2

#define CONFIG_SYS_FSL_ESDHC_ADDR		0

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_SYS_I2C_SPEED		100000

/* USB configs */
#define CONFIG_USBD_HS

#define CONFIG_USB_GADGET_VBUS_DRAW 2

#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2

#endif
