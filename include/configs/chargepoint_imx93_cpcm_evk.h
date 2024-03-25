/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2022 NXP
 */

#ifndef __IMX93_CPCM_EVK_H
#define __IMX93_CPCM_EVK_H

#include <linux/sizes.h>
#include <linux/stringify.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#define CFG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#define PHY_ANEG_TIMEOUT 20000

#define BOOTENV

#define CONSOLE_DEV	"ttyLP0"

#define CFG_MFG_ENV_SETTINGS \
	"console=" CONSOLE_DEV "\0" \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"reboot=h earlycon ignore_loglevel debug " \
		"clk_ignore_unused root=/dev/ram " \
		"\0" \
	"bootcmd_mfg=run mfgtool_args; " \
		"echo \"Run fastboot ...\"; fastboot 0; "  \
		"\0" \

/* Initial environment variables */
#define CFG_EXTRA_ENV_SETTINGS		\
	CFG_MFG_ENV_SETTINGS \
	"bootenvpart=0:1\0" \
	"bootenv=uboot.env\0" \
	"bootfile=fitImage\0" \
	"bootparta=0:2\0" \
	"bootpartb=0:3\0" \
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


/* Link Definitions */

#define CFG_SYS_INIT_RAM_ADDR        0x80000000
#define CFG_SYS_INIT_RAM_SIZE        0x200000

#define CFG_SYS_SDRAM_BASE           0x80000000
#define PHYS_SDRAM                   0x80000000
#define PHYS_SDRAM_SIZE			     0x80000000 /* 2GB DDR */

#define CFG_SYS_FSL_USDHC_NUM	2

/* Using ULP WDOG for reset */
#define WDOG_BASE_ADDR          WDG3_BASE_ADDR

#endif
