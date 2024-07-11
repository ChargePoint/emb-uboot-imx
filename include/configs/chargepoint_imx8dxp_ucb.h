/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018-2019 NXP
 */

#ifndef __IMX8DXP_UCB_H
#define __IMX8DXP_UCB_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#include "imx_env.h"

#define CFG_SYS_FSL_ESDHC_ADDR       0

/* Physical Memory Map */
#define CFG_SYS_SDRAM_BASE              0x080000000
#define PHYS_SDRAM_1                    0x080000000
#define PHYS_SDRAM_1_SIZE               0x080000000	/* 2 GB */
#define PHYS_SDRAM_2                    0x880000000
#define PHYS_SDRAM_2_SIZE               0x040000000	/* 1 GB */
#define CFG_SYS_FSL_USDHC_NUM	2

/* Serial */
#define CONSOLE_DEV	"ttyLP0"

/* Networking */
#define IMX_FEC_BASE			0x5B040000
#define CFG_FEC_MXC_PHYADDR		0x0

#define FEC_QUIRK_ENET_MAC
#define PHY_ANEG_TIMEOUT 20000

/* MT35XU512ABA1G12 has only one Die, so QSPI0 B won't work */
#ifdef CONFIG_FSL_FSPI
#define FSL_FSPI_FLASH_SIZE		SZ_64M
#define FSL_FSPI_FLASH_NUM		1
#define FSPI0_BASE_ADDR			0x5d120000
#define FSPI0_AMBA_BASE			0
#define CONFIG_SYS_FSL_FSPI_AHB
#endif

#define CFG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"reboot=h earlyprintk=serial ignore_loglevel debug " \
		"clk_ignore_unused root=/dev/ram " \
		"\0" \
	"bootcmd_mfg=run mfgtool_args; " \
		"echo \"Run fastboot ...\"; fastboot 0; "  \
		"\0" \


#define CFG_EXTRA_ENV_SETTINGS \
	CFG_MFG_ENV_SETTINGS \
	"console=" CONSOLE_DEV "\0" \
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
					"display fitconfig energystar " \
					"trybootpart bootpart bootlabel; " \
		"elif ext4load mmc ${bootenvpart} " \
			"${loadaddr} ${bootenv}-backup; then " \
				"env import -c ${loadaddr} ${filesize} " \
					"display fitconfig energystar " \
					"bootpart bootlabel; " \
				"env export -c ${loadaddr} " \
					"display fitconfig energystar " \
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
				"display fitconfig energystar " \
				"trybootpart bootpart bootlabel; " \
			"ext4write mmc ${bootenvpart} ${loadaddr} " \
				"/${bootenv} ${filesize}; " \
			"part uuid mmc ${_trybootpart} bootuuid; " \
			"if test \"${energystar}\" = true; then " \
				"setenv -f bootargs_append ${bootargs_append} energystar=1; " \
			"fi; " \
			"setenv bootargs reboot=h ${bootargs_secureboot} " \
				"console=${console} " \
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
		"setenv -f bootmarg ${loadaddr}${extension_fitconfig}; " \
		"run mmctryboot; " \
		"if test -n ${bootpart} && test ${bootpart} != none; then " \
			"setenv -f _bootpart ${bootpart}; " \
		"fi; " \
		"echo Booting ${bootfile} from mmc ${_bootpart} ...; " \
		"part uuid mmc ${_bootpart} bootuuid; " \
		"if test \"${energystar}\" = true; then " \
			"setenv -f bootargs_append ${bootargs_append} energystar=1; " \
		"fi; " \
		"setenv bootargs reboot=h ${bootargs_secureboot} " \
			"console=${console} " \
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
		"if test \"${energystar}\" = true; then " \
			"setenv -f bootargs_append ${bootargs_append} energystar=1; " \
		"fi; " \
		"setenv bootargs reboot=h ${bootargs_secureboot} " \
			"console=${console} " \
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

#endif /* __IMX8DXP_UCB_H */
