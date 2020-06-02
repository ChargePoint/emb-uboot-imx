// SPDX-License-Identifier: GPL-2.0+
/*
 * ChargePoint UCB U-Boot
 *
 * Portions Copyright 2018 NXP
 */
#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <netdev.h>
#include <fsl_ifc.h>
#include <fdt_support.h>
#include <linux/libfdt.h>
#include <environment.h>
#include <fsl_esdhc.h>
#include <i2c.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <dm.h>
#include <asm/arch/snvs_security_sc.h>
#include <imx8_hsio.h>
#include <usb.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/video.h>
#include <asm/arch/video_common.h>
#include <power-domain.h>
#include <asm/arch/lpcg.h>
#include <bootm.h>

#ifdef CONFIG_USB_CDNS3_GADGET
#include <cdns3-uboot.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

int board_early_init_f(void)
{
	sc_ipc_t ipcHndl = 0;
	sc_err_t sciErr = 0;

	ipcHndl = gd->arch.ipc_channel_handle;

	/* Power up UART0 */
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_UART_0, SC_PM_PW_MODE_ON);
	if (sciErr != SC_ERR_NONE)
		return 0;

	/* Set UART0 clock root to 80 MHz */
	sc_pm_clock_rate_t rate = 80000000;
	sciErr = sc_pm_set_clock_rate(ipcHndl, SC_R_UART_0, 2, &rate);
	if (sciErr != SC_ERR_NONE)
		return 0;

	/* Enable UART0 clock root */
	sciErr = sc_pm_clock_enable(ipcHndl, SC_R_UART_0, 2, true, false);
	if (sciErr != SC_ERR_NONE)
		return 0;

	LPCG_AllClockOn(LPUART_0_LPCG);

	return 0;
}

#ifdef CONFIG_FSL_HSIO
static void imx8qxp_hsio_initialize(void)
{
	struct power_domain pd;
	int ret;

	if (!power_domain_lookup_name("hsio_pcie1", &pd)) {
		ret = power_domain_on(&pd);
		if (ret)
			printf("hsio_pcie1 Power up failed! (error = %d)\n", ret);
	}

	if (!power_domain_lookup_name("hsio_gpio", &pd)) {
		ret = power_domain_on(&pd);
		if (ret)
			printf("hsio_gpio Power up failed! (error = %d)\n", ret);
	}

	LPCG_AllClockOn(HSIO_PCIE_X1_LPCG);
	LPCG_AllClockOn(HSIO_PHY_X1_LPCG);
	LPCG_AllClockOn(HSIO_PHY_X1_CRR1_LPCG);
	LPCG_AllClockOn(HSIO_PCIE_X1_CRR3_LPCG);
	LPCG_AllClockOn(HSIO_MISC_LPCG);
	LPCG_AllClockOn(HSIO_GPIO_LPCG);
}

void pci_init_board(void)
{
	imx8qxp_hsio_initialize();

	/* test the 1 lane mode of the PCIe A controller */
	mx8qxp_pcie_init();
}
#endif

#ifdef CONFIG_USB

#ifdef CONFIG_USB_CDNS3_GADGET
static struct cdns3_device cdns3_device_data = {
	.none_core_base = 0x5B110000,
	.xhci_base = 0x5B130000,
	.dev_base = 0x5B140000,
	.phy_base = 0x5B160000,
	.otg_base = 0x5B120000,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 1,
};

int usb_gadget_handle_interrupts(int index)
{
	cdns3_uboot_handle_interrupt(index);

	return 0;
}
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

#ifdef CONFIG_USB_CDNS3_GADGET
	struct power_domain pd;
	if (index != 1 || init == USB_INIT_HOST)
		return ret;

	/* Power on usb */
	if (!power_domain_lookup_name("conn_usb2", &pd)) {
		ret = power_domain_on(&pd);
		if (ret) {
			printf("conn_usb2 Power up failed! (error = %d)\n", ret);
			return ret;
		}
	}

	if (!power_domain_lookup_name("conn_usb2_phy", &pd)) {
		ret = power_domain_on(&pd);
		if (ret) {
			printf("conn_usb2_phy Power up failed! (error = %d)\n", ret);
			return ret;
		}
	}
	ret = cdns3_uboot_init(&cdns3_device_data);
	printf("%d cdns3_uboot_initmode %d\n", index, ret);
#endif

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

#ifdef CONFIG_USB_CDNS3_GADGET
	struct power_domain pd;
	if (index != 1 || init == USB_INIT_HOST)
		return ret;

	cdns3_uboot_exit(1);

	/* Power off usb */
	if (!power_domain_lookup_name("conn_usb2", &pd)) {
		ret = power_domain_off(&pd);
		if (ret)
			printf("conn_usb2 Power down failed! (error = %d)\n", ret);
	}

	if (!power_domain_lookup_name("conn_usb2_phy", &pd)) {
		ret = power_domain_off(&pd);
		if (ret)
			printf("conn_usb2_phy Power down failed! (error = %d)\n", ret);
	}
#endif

	return ret;
}
#endif // CONFIG_USB

int board_init(void)
{
	return 0;
}

void board_quiesce_devices()
{
	const char *power_on_devices[] = {
		"dma_lpuart0",

		/* HIFI DSP boot */
		"audio_sai0",
		"audio_ocram",
	};

	power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(ulong addr)
{
	puts("SCI reboot request");
	sc_pm_reboot(SC_IPC_CH, SC_PM_RESET_TYPE_COLD);
	while (1)
		putc('.');
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#if defined(CONFIG_VIDEO_IMXDPUV1)
static void enable_lvds(struct display_info_t const *dev)
{
	struct gpio_desc desc;
	int ret;

	/* MIPI_DSI0_EN on IOEXP 0x1a port 6, MIPI_DSI1_EN on IOEXP 0x1d port 7 */
	ret = dm_gpio_lookup_name("gpio@1a_6", &desc);
	if (ret)
		return;

	ret = dm_gpio_request(&desc, "lvds0_en");
	if (ret)
		return;

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	display_controller_setup((PS2KHZ(dev->mode.pixclock) * 1000));
	lvds_soc_setup(dev->bus, (PS2KHZ(dev->mode.pixclock) * 1000));
	lvds_configure(dev->bus);
	lvds2hdmi_setup(13);
}

struct display_info_t const displays[] = {{
	.bus	= 0, /* LVDS0 */
	.addr	= 0, /* LVDS0 */
	.pixfmt	= IMXDPUV1_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "IT6263", /* 720P60 */
		.refresh        = 60,
		.xres           = 800,
		.yres           = 600,
		.pixclock       = 13468, /* 74250000 */
		.left_margin    = 110,
		.right_margin   = 220,
		.upper_margin   = 5,
		.lower_margin   = 20,
		.hsync_len      = 40,
		.vsync_len      = 5,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
	}
} };

size_t display_count = ARRAY_SIZE(displays);

#endif /* CONFIG_VIDEO_IMXDPUV1 */

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif
