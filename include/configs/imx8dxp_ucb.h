/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018-2019 NXP
 */

#ifndef __IMX8DXP_UCB_H
#define __IMX8DXP_UCB_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#include "version.h"
#include "imx_env.h"

#define CONFIG_SERIAL_TAG
#define CONFIG_REMAKE_ELF

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_ARCH_MISC_INIT

/* Flat Device Tree Definitions */
#define CONFIG_OF_BOARD_SETUP
#define CONFIG_OF_SYSTEM_SETUP

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN           (40 * SZ_1M)

#define CONFIG_SYS_BOOTM_LEN            (64 << 20)
#define CONFIG_SYS_XIMG_LEN CONFIG_SYS_BOOTM_LEN

/* Physical Memory Map */
#define CONFIG_SYS_SDRAM_BASE           0x80000000
#define CONFIG_NR_DRAM_BANKS            4
#define PHYS_SDRAM_1                    0x080000000
#define PHYS_SDRAM_1_SIZE               0x080000000	/* 2 GB */
#define PHYS_SDRAM_2                    0x880000000
#define PHYS_SDRAM_2_SIZE               0x040000000	/* 1 GB */

#define CONFIG_SYS_MEMTEST_START        0xA0000000
#define CONFIG_SYS_MEMTEST_END \
	(CONFIG_SYS_MEMTEST_START + (PHYS_SDRAM_1_SIZE >> 2))

#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR       0
#define USDHC1_BASE_ADDR                0x5B010000
#define USDHC2_BASE_ADDR                0x5B020000
#define CONFIG_SUPPORT_EMMC_BOOT

#define CONFIG_MXC_GPIO

/* Link Definitions */
#define CONFIG_LOADADDR                 0x98000000
#define CONFIG_SYS_LOAD_ADDR            CONFIG_LOADADDR
#define CONFIG_SYS_INIT_SP_ADDR         0x80200000

/* Environment organization */
#define CONFIG_ENV_SIZE                 (8 * SZ_1K)
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define FDT_SEQ_MACADDR_FROM_ENV

/* Default environment is in mmcblk0boot1 */
#define CONFIG_SYS_MMC_ENV_DEV          0   /* mmcblk0 */
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET               0   /* start of mmcblk0boot1 */
#ifndef CONFIG_SYS_MMC_ENV_PART
#define CONFIG_SYS_MMC_ENV_PART         2   /* start of mmcblk0boot1 */
#endif
#endif
#define CONFIG_SYS_MMC_IMG_LOAD_PART    1

/* Serial */
#define CONSOLE_DEV	"ttyLP0"

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2      "UCB> "
#define CONFIG_SYS_CBSIZE               2048
#define CONFIG_SYS_MAXARGS              64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE \
	(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

/* Generic Timer Definitions */
#define COUNTER_FREQUENCY               8000000	/* 8MHz */

/* USB Config */
#define CONFIG_USB_STORAGE
#define CONFIG_USBD_HS
#define CONFIG_USB_EHCI_HCD
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

/* USB OTG controller configs */
#ifdef CONFIG_USB_EHCI_HCD
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC           (PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

/* Networking */
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE                    0x5B040000
#define CONFIG_FEC_MXC_PHYADDR          0x0
#define CONFIG_ETHPRIME                 "eth0"
#else
#define IMX_FEC_BASE                    0x5B050000
#define CONFIG_FEC_MXC_PHYADDR          0x1
#define CONFIG_ETHPRIME                 "eth1"
#endif

/* ENET0 MDIO are shared */
#define CONFIG_FEC_MXC_MDIO_BASE        0x5B040000
#define CONFIG_FEC_XCV_TYPE             RGMII
#define FEC_QUIRK_ENET_MAC

#ifndef CONFIG_LIB_RAND
#define CONFIG_LIB_RAND
#endif
#define CONFIG_NET_RANDOM_ETHADDR

/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IMXDPUV1
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

/*
 * Resource checking produces unwanted device tree warnings.
 * Happened after UCB switched to 5.4 kernel dt bindings.
 */
#define CONFIG_SKIP_RESOURCE_CHECING

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
				"resetreason=${resetreason}; " \
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
			"resetreason=${resetreason}; " \
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
			"root=PARTUUID=${bootuuid} rootwait rw; " \
		"ext4load mmc ${_bootpart} ${loadaddr} ${bootfile} && " \
			"bootm ${bootmarg}; " \
	"\0"


#define CONFIG_BOOTCOMMAND \
	"run mmcboot"

#define CONFIG_BOOTARGS \
	"console=" CONSOLE_DEV ",115200 earlyprintk=serial ignore_loglevel"

#endif /* __IMX8DXP_UCB_H */
