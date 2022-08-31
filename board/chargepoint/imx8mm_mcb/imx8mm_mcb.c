// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 ChargePoint, Inc.
 *
 * Portions Copyright 2018 NXP
 */
#include <common.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/io.h>
#include "../common/fitimage_keys.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	return 0;
}

int board_init(void)
{
	setup_fitimage_keys();
	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "MCB");
	env_set("board_rev", "iMX8MM");
#endif

#if !defined(CONFIG_FASTBOOT)
	/* set an environment that this is a secure boot */
	if (hab_is_enabled()) {
		env_set("bootargs_secureboot", "uboot-secureboot");
	}
#endif

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	uint32_t uid_high, uid_low;

	/*
	 * i.MX8M Configuration and Manufacturing Info
	 *   OCOTP_CFG0: OTP Bank 0, word 1
	 *   OCOTP_CFG1: OTP Bank 0, word 2
	 */
	do {
		struct ocotp_regs *ocotp =
			(struct ocotp_regs *)OCOTP_BASE_ADDR;
		struct fuse_bank *bank = &ocotp->bank[0];
		struct fuse_bank0_regs *fuse =
			(struct fuse_bank0_regs *)bank->fuse_regs;

		uid_high = readl(&fuse->uid_high);
		uid_low = readl(&fuse->uid_low);
	} while(0);

	if (!env_get("serial#")) {
		char serial_str[17];

		snprintf(serial_str, sizeof(serial_str),
			 "%08x%08x", uid_high, uid_low);
		env_set("serial#", serial_str);

		fdt_root(blob);
	}

	return 0;
}
#endif
