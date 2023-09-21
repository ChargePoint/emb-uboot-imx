/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018-2019 NXP
 */

#ifndef __IMX8QXP_MEK_H
#define __IMX8QXP_MEK_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#include "imx_env.h"

#define CONFIG_SERIAL_TAG
#define CONFIG_REMAKE_ELF

// Item moved to defconfig - #define CONFIG_BOARD_EARLY_INIT_F

/* Flat Device Tree Definitions */
// moved to defconfig - #define CONFIG_OF_BOARD_SETUP

// moved to defconfig - #define CONFIG_CMD_READ
// moved to defconfig - #undef CONFIG_CMD_CRC32

/* Size of malloc() pool */
// moved to defconfig #define CONFIG_SYS_MALLOC_LEN           (40 * SZ_1M)

// 64 << 20 is equal to 0x4000000
// Compiler was complaining about redefinition despite them being the same.
#define CONFIG_SYS_BOOTM_LEN            0x4000000
#define CONFIG_SYS_XIMG_LEN CONFIG_SYS_BOOTM_LEN

/* Physical Memory Map */
#define CFG_SYS_SDRAM_BASE              0x080000000
#define PHYS_SDRAM_1                    0x080000000
#define PHYS_SDRAM_1_SIZE               0x080000000	/* 2 GB */
#define PHYS_SDRAM_2                    0x880000000
#define PHYS_SDRAM_2_SIZE               0x040000000	/* 1 GB */

/* RMW - moved to defconfig but without the calculations
#define CONFIG_SYS_MEMTEST_START        0xA0000000
#define CONFIG_SYS_MEMTEST_END \
	(CONFIG_SYS_MEMTEST_START + (PHYS_SDRAM_1_SIZE >> 2))
*/

/* RMW TODO - these items need reviewed with Charles. I think unnecessary items.
#define CONFIG_SYS_FSL_ESDHC_ADDR       0
#define USDHC1_BASE_ADDR                0x5B010000
#define USDHC2_BASE_ADDR                0x5B020000
*/

/* Link Definitions */
// TODO RMW - do we still need LOADADDR? If so, should it be CONFIG_SYS_LOAD_ADDR instead of value?
// RMW removing - not used in u-boot - #define CONFIG_LOADADDR                 0x98000000
// went to defconfig as CUSTOM_SYS_INIT_SP_ADDR - #define CONFIG_SYS_INIT_SP_ADDR         0x80200000

/* Environment organization */
// moved to defconfig #define CONFIG_ENV_OVERWRITE
// moved to defconfig - #define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

/* moved to defconfig
#define CONFIG_PCIE_IMX
#define CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
*/

/* Default environment is in mmcblk0boot1 */
// moved to defconfig #define CONFIG_SYS_MMC_ENV_DEV          0   /* mmcblk0 */
// RMW TODO this isn't found in u-boot anymore. #define CONFIG_SYS_MMC_IMG_LOAD_PART    1

#define CONFIG_SYS_FSL_USDHC_NUM	2

/* Serial */
#define CONSOLE_DEV	"ttyLP0"
#define CONFIG_BAUDRATE			115200

/* Monitor Command Prompt */
/* RMW Moved to defconfig
#define CONFIG_SYS_PROMPT_HUSH_PS2      "UCB> "
#define CONFIG_SYS_CBSIZE               2048
#define CONFIG_SYS_MAXARGS              64
#define CONFIG_SYS_PBSIZE \
	(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
*/
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

/* Generic Timer Definitions */
#define COUNTER_FREQUENCY               8000000	/* 8MHz */

#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

/* USB OTG controller configs */
#ifdef CONFIG_USB_EHCI_HCD
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC           (PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

/* Networking */
#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE                    0x5B040000
#define CONFIG_FEC_MXC_PHYADDR          0x0
#define CONFIG_ETHPRIME                 "eth0"
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE                    0x5B050000
#define CONFIG_FEC_MXC_PHYADDR          0x1
#define CONFIG_ETHPRIME                 "eth1"
#endif

#define CONFIG_FEC_XCV_TYPE             RGMII
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
			"${bootargs_append}; " \
		"ext4load mmc ${_bootpart} ${loadaddr} ${bootfile} && " \
			"bootm ${bootmarg}; " \
	"\0"


#define CONFIG_BOOTCOMMAND \
	"run mmcboot"

#define CONFIG_BOOTARGS \
	"console=" CONSOLE_DEV ",115200 earlyprintk=serial ignore_loglevel"

#endif /* __IMX8QXP_MEK_H */
