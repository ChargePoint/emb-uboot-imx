/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6DLSABRE_COMMON_PBC_CONFIG_H
#define __MX6DLSABRE_COMMON_PBC_CONFIG_H

#include "mx6_common.h"
#include "version.h"
#include "imx_env.h"

#include <asm/arch/imx-regs.h>
#include <asm/arch-mx6/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_MXC_UART

#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM        2
#define CONFIG_SYS_MMC_ENV_DEV          0       /* MMC0 */
#define CONFIG_SYS_MMC_ENV_PART         2       /* eMMB boot2 parition */

#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME			"eth0"
#else
#define CONFIG_ETHPRIME			"FEC"
#endif
#define CONFIG_FEC_MXC_PHYADDR		1

#define CONFIG_PHYLIB

#ifdef CONFIG_MX6S
#define SYS_NOSMP "nosmp"
#else
#define SYS_NOSMP
#endif

#ifdef CONFIG_SUPPORT_EMMC_BOOT
#define EMMC_ENV \
	"emmcdev=0\0" \
	"update_emmc_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if ${get_cmd} ${update_sd_firmware_filename}; then " \
			"if mmc dev ${emmcdev} 1; then "	\
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0"
#else
#define EMMC_ENV ""
#endif

#define CONFIG_OTA_ENV_SETTINGS \
	"bank_curr=2\0" \
	"do_usb_ota=fatload usb 0:1 ${loadaddr} OTA_uImage.imx6q; setenv bootargs console=${console},${baudrate}  noinitrd mmcblk.perdev_minors=16 ota_device=/dev/sda1 ota_filename=ota.tar.xz; bootm ${loadaddr}\0" \
    "fix_partitions=echo fake fix_partitions...\0" \

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_OTA_ENV_SETTINGS \
	"version_uboot=" U_BOOT_VERSION "\0" \
	"epdc_waveform=epdc_splash.bin\0" \
	"script=boot.scr\0" \
	"image=/boot/zImage\0" \
	"fdt_file=/boot/imx6dp-chargepoint-pbc.dtb\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"mmcautodetect=yes\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	EMMC_ENV	  \
	"smp=" CONFIG_SYS_NOSMP "\0"\
	"mmcargs=setenv bootargs console=${console},${baudrate} mmcblk.perdev_minors=16 reset_cause=${reset_cause} ${smp}" \
		"root=/dev/mmcblk0p${bank_curr} rootwait rw init=/sbin/init.chargepoint\0" \
	"loadbootscript=" \
		"fatload mmc 0:${bank_curr} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=ext4load mmc 0:${bank_curr} ${loadaddr} ${image}\0" \
	"loadfdt=ext4load mmc 0:${bank_curr} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} ${smp} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	"run fix_partitions; if test ${bank_curr} -ne 2 && test ${bank_curr} -ne 3; then " \
		"echo WARN: Bad uboot env detected. Restoring default env; " \
		"env default -a -f; saveenv; " \
	"fi; " \
	"mmc dev ${mmcdev};" \
	"usb start; if fatsize usb 0:1 OTA_uImage.imx6q; then run do_usb_ota; fi; " \
	"if mmc rescan; then " \
		"if run loadimage; then run mmcboot; else env default -a -f; saveenv; reset; fi; " \
	"fi; "

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
//#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#ifndef CONFIG_SYS_NOSMP
#define CONFIG_SYS_NOSMP
#endif

#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_SPEED 20000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

#ifdef CONFIG_MTD_NOR_FLASH
#define CONFIG_SYS_FLASH_BASE           WEIM_ARB_BASE_ADDR
#define CONFIG_SYS_FLASH_SECT_SIZE      (128 * 1024)
#define CONFIG_SYS_MAX_FLASH_BANKS 1    /* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT 256   /* max number of sectors on one chip */
#define CONFIG_SYS_FLASH_CFI            /* Flash memory is CFI compliant */
#define CONFIG_FLASH_CFI_DRIVER         /* Use drivers/cfi_flash.c */
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE /* Use buffered writes*/
#define CONFIG_SYS_FLASH_EMPTY_INFO
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(8 * 64 * 1024)
#endif

/* I2C Configs */
#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#endif
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000
#endif

/* PMIC */
#ifndef CONFIG_DM_PMIC
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR 0x08
#endif

#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx6sabreandroid_common.h"
#else
#define CONFIG_USBD_HS

#endif /* CONFIG_ANDROID_SUPPORT */
#endif                         /* __MX6DLSABRE_COMMON_PBC_CONFIG_H */
