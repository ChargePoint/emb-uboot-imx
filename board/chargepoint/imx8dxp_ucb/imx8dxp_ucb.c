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

#include "../common/fitimage_keys.h"

DECLARE_GLOBAL_DATA_PTR;


/*
 * Rely on the device-tree to setup the peripherals, but setup the
 * uart0 pads to get the initial prints before the DM code kicks in
 */
#define UART_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | \
			 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | \
			 (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

static iomux_cfg_t uart0_pads[] = {
	SC_P_UART0_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_UART0_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_early_init_f(void)
{
	sc_err_t err;
	uint16_t lc;
	sc_ipc_t ipcHndl;

	ipcHndl = gd->arch.ipc_channel_handle;

	/* Determine the security state of the chip (OEM closed) */
	err = sc_seco_chip_info(ipcHndl, &lc, NULL, NULL, NULL);
	if ((err == SC_ERR_NONE) && (lc == 0x80)) {
		/* HAB is in OEM closed, so disable the serial console */
		gd->flags |= (GD_FLG_SILENT | GD_FLG_DISABLE_CONSOLE);
	}

	/* Power up UART0 */
	err = sc_pm_set_resource_power_mode(ipcHndl, SC_R_UART_0,
					    SC_PM_PW_MODE_ON);
	if (err != SC_ERR_NONE)
		return 0;

	/* Set UART0 clock root to 80 MHz */
	sc_pm_clock_rate_t rate = 80000000;
	err = sc_pm_set_clock_rate(ipcHndl, SC_R_UART_0, 2, &rate);
	if (err != SC_ERR_NONE)
		return 0;

	/* Enable UART0 clock root */
	err = sc_pm_clock_enable(ipcHndl, SC_R_UART_0, 2, true, false);
	if (err != SC_ERR_NONE)
		return 0;

	LPCG_AllClockOn(LPUART_0_LPCG);

	imx8_iomux_setup_multiple_pads(uart0_pads, ARRAY_SIZE(uart0_pads));

	return 0;
}

#ifdef CONFIG_FSL_HSIO
static void imx8qxp_hsio_initialize(void)
{
	struct power_domain pd;
	int ret;

	if (!power_domain_lookup_name("hsio_pcie1", &pd)) {
		ret = power_domain_on(&pd);
		if (ret) {
			printf("Power up hsio_pcie1 (error = %d)\n", ret);
		}
	}

	if (!power_domain_lookup_name("hsio_gpio", &pd)) {
		ret = power_domain_on(&pd);
		if (ret) {
			printf("Power up hsio_gpio (error = %d)\n", ret);
		}
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
			printf("Power up conn_usb2 (error = %d)\n", ret);
			return ret;
		}
	}

	if (!power_domain_lookup_name("conn_usb2_phy", &pd)) {
		ret = power_domain_on(&pd);
		if (ret) {
			printf("Power up conn_usb2_phy (error = %d)\n", ret);
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
		if (ret) {
			printf("Power down conn_usb2 (error = %d)\n", ret);
		}
	}

	if (!power_domain_lookup_name("conn_usb2_phy", &pd)) {
		ret = power_domain_off(&pd);
		if (ret) {
			printf("Power down conn_usb2_phy (error = %d)\n", ret);
		}
	}
#endif

	return ret;
}
#endif // CONFIG_USB

int board_init(void)
{

	setup_fitimage_keys();

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
	for (;;) {
		putc('.');
	}
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int board_late_init(void)
{
	sc_err_t err;
	uint16_t lc;
	sc_ipc_t ipcHndl;

	ipcHndl = gd->arch.ipc_channel_handle;

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	/* Determine the security state of the chip (OEM closed) */
	err = sc_seco_chip_info(ipcHndl, &lc, NULL, NULL, NULL);
	if (err == SC_ERR_NONE) {
		switch (lc) {
		default:
		case 0x1: /* Pristine */
		case 0x2: /* Fab */
		case 0x8: /* Open */
		case 0x20: /* NXP closed */
		case 0x100: /* Partial field return */
		case 0x200: /* Full field return */
			break;

		case 0x80: /* OEM closed */
#if defined(CONIG_NOT_UUU_BUILD)
			env_set("bootargs_secureboot", "uboot-secureboot");
#endif
			break;
		}
	}

	return 0;
}

#if defined(CONFIG_VIDEO_IMXDPUV1)
static void enable_lvds(struct display_info_t const *dev)
{
	struct gpio_desc desc;
	int ret;

	/*
	 * MIPI_DSI0_EN on IOEXP 0x1a port 6
	 * MIPI_DSI1_EN on IOEXP 0x1d port 7
	 */
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
/* define the get mac function here to avoid including fec_mxc.h */
void imx_get_mac_from_fuse(int dev_id, unsigned char *mac);

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
		sc_ipc_t ipcHndl;

		ipcHndl = gd->arch.ipc_channel_handle;

#define FUSE_UNIQUE_ID_LOW	16
#define FUSE_UNIQUE_ID_HIGH	17

		word = FUSE_UNIQUE_ID_LOW;
		err = sc_misc_otp_fuse_read(ipcHndl, word, &uid_low);
		if (err != SC_ERR_NONE) {
			printf("%s fuse %d read error: %d\n",
				__func__, word, err);
			goto out;
		}
		word = FUSE_UNIQUE_ID_HIGH;
		err = sc_misc_otp_fuse_read(ipcHndl, word, &uid_high);
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

	if (!env_get("ethaddr") && !env_get("eth1addr")) {
		u8 mac_addr[6];
		char mac_str[ARP_HLEN_ASCII + 1];

		/* MAC0 */
		imx_get_mac_from_fuse(0, mac_addr);

		debug("imx fuse 0 mac addr %x:%x:%x:%x:%x:%x\n",
			mac_addr[0], mac_addr[1], mac_addr[2],
			mac_addr[3], mac_addr[4], mac_addr[5]);
		if (!is_valid_ethaddr(mac_addr)) {
			mac_addr[0] =
				((uid_low >> 24) ^ (uid_low >> 16)) & 0xff;
			mac_addr[1] =
				((uid_low >> 8) ^ (uid_low >> 0)) & 0xff;
			mac_addr[2] = uid_low >> 24;
			mac_addr[3] = uid_low >> 16;
			mac_addr[4] = uid_low >> 8;
			mac_addr[5] = uid_low >> 0;

			mac_addr[0] &= 0xfe; /* clear multicast bit */
			mac_addr[0] |= 0x02; /* set local assignment bit */
			debug("ocotp uid %x:%x -> macaddr %x:%x:%x:%x:%x:%x\n",
				uid_high, uid_low,
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);
		}

		snprintf(mac_str, sizeof(mac_str), "%x:%x:%x:%x:%x:%x",
			 mac_addr[0], mac_addr[1], mac_addr[2],
			 mac_addr[3], mac_addr[4], mac_addr[5]);
		debug("ethaddr - %s\n", mac_str);
		env_set("ethaddr", mac_str);

		/* MAC1 */
		imx_get_mac_from_fuse(1, mac_addr);

		debug("imx fuse 1 mac addr %x:%x:%x:%x:%x:%x\n",
			mac_addr[0], mac_addr[1], mac_addr[2],
			mac_addr[3], mac_addr[4], mac_addr[5]);
		if (!is_valid_ethaddr(mac_addr)) {
			mac_addr[0] =
				((uid_high >> 24) ^ (uid_high >> 16)) & 0xff;
			mac_addr[1] =
				((uid_high >> 8) ^ (uid_high >> 0)) & 0xff;
			mac_addr[2] = uid_high >> 24;
			mac_addr[3] = uid_high >> 16;
			mac_addr[4] = uid_high >> 8;
			mac_addr[5] = uid_high >> 0;

			mac_addr[0] &= 0xfe; /* clear multicast bit */
			mac_addr[0] |= 0x02; /* set local assignment bit */
			debug("ocotp uid %x:%x -> macaddr %x:%x:%x:%x:%x:%x\n",
				uid_high, uid_low,
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);
		}

		snprintf(mac_str, sizeof(mac_str), "%x:%x:%x:%x:%x:%x",
			 mac_addr[0], mac_addr[1], mac_addr[2],
			 mac_addr[3], mac_addr[4], mac_addr[5]);
		debug("eth1addr - %s\n", mac_str);
		env_set("eth1addr", mac_str);

		fdt_fixup_ethernet(blob);
	}

out:
	return 0;
}
#endif
