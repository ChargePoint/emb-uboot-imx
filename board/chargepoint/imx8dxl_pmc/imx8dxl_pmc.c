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
#include <asm/setup.h>
#include <asm/bootm.h>
#include "../common/fitimage_keys.h"

DECLARE_GLOBAL_DATA_PTR;


/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

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

static int dm_get_gpio(const char *name, const char *label)
{
	struct gpio_desc desc;
	int ret;

	ret = dm_gpio_lookup_name(name,&desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, name, ret);
		return -1;
	}
	ret = dm_gpio_request(&desc, label);
	if (ret) {
		printf("%s dm_gpio_request for %s failed ret = %d\n",
		       __func__, label, ret);
		return -1;
	}
	return dm_gpio_get_value(&desc);
}

static inline void board_gpio_init(void) {}

int board_early_init_f(void)
{
	sc_pm_clock_rate_t rate = SC_80MHZ;
	int ret;

	do {
		sc_err_t err;
		uint16_t lc;

		/* Determine the security state of the chip (OEM closed) */
		err = sc_seco_chip_info(-1, &lc, NULL, NULL, NULL);
		if ((err == SC_ERR_NONE) && (lc == 0x80)) {
			/* OEM closed, so disable the serial console */
			gd->flags |= (GD_FLG_SILENT | GD_FLG_DISABLE_CONSOLE);
		}
	} while(0);

	/* Set UART0 clock root to 80 MHz */
	ret = sc_pm_setup_uart(SC_R_UART_0, rate);
	if (ret)
		return ret;

	setup_iomux_uart();

	return 0;
}

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

	imx8_power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(void)
{
	/* TODO */
}

int board_late_init(void)
{
	sc_err_t err;
	uint16_t lc;

	build_info();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "PMC");
	env_set("board_rev", "iMX8DXL");
#endif

#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#else
	env_set("sec_boot", "no");
#endif

	/* Determine the security state of the chip (OEM closed) */
	err = sc_seco_chip_info(-1, &lc, NULL, NULL, NULL);
	if (err == SC_ERR_NONE) {
		switch (lc) {
		default:
		case 0x1:   /* Pristine */
		case 0x2:   /* Fab */
		case 0x8:   /* Open */
		case 0x20:  /* NXP closed */
		case 0x100: /* Partial field return */
		case 0x200: /* Full field return */
			break;

		case 0x80:  /* OEM closed */
			/* set an environment that this is a secure boot */
			env_set("bootargs_secureboot", "uboot-secureboot");
			break;
		}
	} else {
		printf("%s: sc_seco_chip_info error %d\n", __func__, err);
	}

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	struct tag_serialnr sn;
	sn.low = sn.high = 0;

	do {
		get_board_serial(&sn);
	} while(0);

	if (!env_get("serial#")) {
		char serial_str[17];

		snprintf(serial_str, sizeof(serial_str),
			 "%08x%08x", sn.high, sn.low);
		env_set("serial#", serial_str);

		fdt_root(blob);
	}

	return 0;
}
#endif
