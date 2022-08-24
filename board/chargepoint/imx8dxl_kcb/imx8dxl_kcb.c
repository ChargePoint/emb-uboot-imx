// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <common.h>
#include <cpu_func.h>
#include <env.h>
#include <errno.h>
#include <init.h>
#include <linux/libfdt.h>
#include <fsl_esdhc_imx.h>
#include <fdt_support.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/snvs_security_sc.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <usb.h>
#include "../common/fitimage_keys.h"

//#include "../../freescale/common/tcpc.h"

DECLARE_GLOBAL_DATA_PTR;

#define ENET_INPUT_PAD_CTRL	((SC_PAD_CONFIG_OD_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_18V_10MA << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define ENET_NORMAL_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_18V_10MA << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define GPMI_NAND_PAD_CTRL	 ((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) \
				  | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | \
			 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | \
			 (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define UART_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | \
			 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | \
			 (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

static iomux_cfg_t uart0_pads[] = {
	SC_P_UART0_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_UART0_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx8_iomux_setup_multiple_pads(uart0_pads, ARRAY_SIZE(uart0_pads));
}

int board_early_init_f(void)
{
	sc_pm_clock_rate_t rate = SC_80MHZ;
	int ret;

	/* Set UART0 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_0, rate);
	if (ret)
		return ret;

	setup_iomux_uart();

	return 0;
}

#if CONFIG_IS_ENABLED(DM_GPIO)
static void board_gpio_init(void)
{
#if defined(CONFIG_DM_VIDEO)
	int ret;
	struct gpio_desc desc;

	/* M40_DEBUG_UART_SEL */
	ret = dm_gpio_lookup_name("gpio@20_3", &desc);
	if (ret) {
		printf("%s lookup gpio@20_3 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "M40_DEBUG_UART_SEL");
	if (ret) {
		printf("%s request M40_DEBUG_UART_SEL failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);

	/* SPI0_SEL */
	ret = dm_gpio_lookup_name("gpio@20_8", &desc);
	if (ret) {
		printf("%s lookup gpio@20_8 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "SPI0_SEL");
	if (ret) {
		printf("%s request SPI0_SEL failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);

	/* UART1_SEL */
	ret = dm_gpio_lookup_name("gpio@20_6", &desc);
	if (ret) {
		printf("%s lookup gpio@20_6 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "UART1_SEL");
	if (ret) {
		printf("%s request UART1_SEL failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);

	/* MUX3_EN */
	ret = dm_gpio_lookup_name("gpio@21_8", &desc);
	if (ret) {
		printf("%s lookup gpio@21_8 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "MUX3_EN");
	if (ret) {
		printf("%s request MUX3_EN failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);

	/* SPI3_CS0_SEL */
	ret = dm_gpio_lookup_name("gpio@20_4", &desc);
	if (ret) {
		printf("%s lookup gpio@20_4 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "SPI3_CS0_SEL");
	if (ret) {
		printf("%s request SPI3_CS0_SEL failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);

	/* SPI3_SEL */
	ret = dm_gpio_lookup_name("gpio@20_7", &desc);
	if (ret) {
		printf("%s lookup gpio@20_7 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "SPI3_SEL");
	if (ret) {
		printf("%s request SPI3_SEL failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);

	/* BL_CTR */
	ret = dm_gpio_lookup_name("gpio@20_5", &desc);
	if (ret) {
		printf("%s lookup gpio@20_5 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "BL_CTR");
	if (ret) {
		printf("%s request BL_CTR failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
#endif
}
#else
static inline void board_gpio_init(void) {}
#endif

#if IS_ENABLED(CONFIG_NET)
#include <miiphy.h>

int board_phy_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

int checkboard(void)
{
#if defined(CONFIG_TARGET_IMX8DXL_DDR3_EVK)
	puts("Board: iMX8DXL DDR3 EVK\n");
#else
	puts("Board: iMX8DXL KCB\n");
#endif

	print_bootinfo();

	return 0;
}

int board_init(void)
{
	setup_fitimage_keys();

	board_gpio_init();

	return 0;
}

void board_quiesce_devices(void)
{
	const char *power_on_devices[] = {
		"dma_lpuart0",
	};

	power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(ulong addr)
{
	/* TODO */
}

int board_late_init(void)
{
	char *fdt_file;
	bool __maybe_unused m4_boot;
	sc_err_t err;
	uint16_t lc;

	build_info();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "KCB");
	env_set("board_rev", "iMX8DXL");
#endif

	env_set("sec_boot", "no");

	fdt_file = env_get("fdt_file");
	m4_boot = check_m4_parts_boot();

	if (fdt_file && !strcmp(fdt_file, "undefined")) {
#if defined(CONFIG_TARGET_IMX8DXL_DDR3_EVK)
		env_set("fdt_file", "imx8dxl-ddr3-evk.dtb");
#else
		if (m4_boot)
			env_set("fdt_file", "imx8dxl-evk-rpmsg.dtb");
		else
			env_set("fdt_file", "imx8dxl-evk.dtb");
#endif
	}

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	/* Determine the security state of the chip (OEM closed) */
	err = sc_seco_chip_info(-1, &lc, NULL, NULL, NULL);

	if (err == SC_ERR_NONE) {
		switch (lc) {
		default:
		case 0x1: 	/* Pristine */
		case 0x2: 	/* Fab */
		case 0x8: 	/* Open */
		case 0x20: 	/* NXP closed */
		case 0x100: /* Partial field return */
		case 0x200: /* Full field return */
			break;

		case 0x80: /* OEM closed */
			/* set an environment that this is a secure boot */
			env_set("bootargs_secureboot", "uboot-secureboot");
			env_set("sec_boot", "yes");
			break;
		}
	}

#if defined(CONFIG_NOT_UUU_BUILD)
	/* set an environment that this is a secure boot */
		env_set("bootargs_secureboot", "uboot-secureboot");
		env_set("sec_boot", "yes");
#endif

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	uint32_t uid_high, uid_low;

	/*
	 * i.MX8 Configuration and Manufacturing Info in OTP fuse array
	 *   UNIQUE_ID LOW: 16
	 *   UNIQUE_ID HI: 17
	 */
	do {
		sc_err_t err;
		uint32_t word;

#define FUSE_UNIQUE_ID_LOW	16
#define FUSE_UNIQUE_ID_HIGH	17

		word = FUSE_UNIQUE_ID_LOW;
		err = sc_misc_otp_fuse_read(-1, word, &uid_low);
		if (err != SC_ERR_NONE) {
			printf("%s fuse %d read error: %d\n",
				__func__, word, err);
			goto out;
		}
		word = FUSE_UNIQUE_ID_HIGH;
		err = sc_misc_otp_fuse_read(-1, word, &uid_high);
		if (err != SC_ERR_NONE) {
			printf("%s fuse %d read error: %d\n",
				__func__, word, err);
			goto out;
		}

#undef FUSE_UNIQUE_ID_LOW
#undef FUSE_UNIQUE_ID_HIGH
	} while(0);

	if (!env_get("serial#")) {
		char serial_str[17];

		snprintf(serial_str, sizeof(serial_str),
			 "%08x%08x", uid_high, uid_low);
		env_set("serial#", serial_str);

		fdt_root(blob);
	}

out:
	return 0;
}
#endif
