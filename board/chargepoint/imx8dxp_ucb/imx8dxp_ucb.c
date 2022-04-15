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
#include "../common/bootargs_util.h"

DECLARE_GLOBAL_DATA_PTR;


/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

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

#ifdef CONFIG_MXC_GPIO
#define GPIO_DBG_LED4	IMX_GPIO_NR(1, 8)
#define GPIO_DBG_LED5	IMX_GPIO_NR(1, 7)
#define GPIO_DBG_LED11	IMX_GPIO_NR(3, 14)

#define GPIO_USBH_RESET	IMX_GPIO_NR(1, 6)

static void set_gpio(int gpio, const char *gpioname, int val)
{
	debug("%s >>>>>>>>\n", __func__);
	gpio_request(gpio, gpioname);
	gpio_direction_output(gpio, val);
	debug("\t%s[%u] -> %#x\n", gpioname, gpio, val);
}

static void board_gpio_init(void)
{
	set_gpio(GPIO_DBG_LED4, "debug_led4", 0);
	set_gpio(GPIO_DBG_LED5, "debug_led5", 0);
	set_gpio(GPIO_DBG_LED11, "debug_led11", 0);

	set_gpio(GPIO_USBH_RESET, "usb5734_reset", 0);
}
#endif

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

#ifdef CONFIG_MXC_GPIO
	board_gpio_init();
#endif

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

static const char *get_reset_reason(void)
{
    sc_pm_reset_reason_t reason;

    sc_err_t ret = sc_pm_reset_reason(SC_IPC_CH, &reason);

    if (ret != SC_ERR_NONE) {
        printf("sc_pm_reset_reason() failed: %i\n", ret);
        return "unknown";
    }

    switch (reason)
    {
        case SC_PM_RESET_REASON_POR:        return "POR";
        case SC_PM_RESET_REASON_JTAG:       return "JTAG";
        case SC_PM_RESET_REASON_SW:         return "SW";
        case SC_PM_RESET_REASON_WDOG:       return "WDOG";
        case SC_PM_RESET_REASON_LOCKUP:     return "LOCKUP";
        case SC_PM_RESET_REASON_SNVS:       return "SNVS";
        case SC_PM_RESET_REASON_TEMP:       return "TEMP";
        case SC_PM_RESET_REASON_MSI:        return "MSI";
        case SC_PM_RESET_REASON_UECC:       return "UECC";
        case SC_PM_RESET_REASON_SCFW_WDOG:  return "SCFW_WDOG";
        case SC_PM_RESET_REASON_ROM_WDOG:   return "ROM_WDOG";
        case SC_PM_RESET_REASON_SECO:       return "SECO";
        case SC_PM_RESET_REASON_SCFW_FAULT: return "SCFW_FAULT";
        default: break;
    }

    return "invalid";
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
			/* set an environment that this is a secure boot */
			env_set("bootargs_secureboot", "uboot-secureboot");
#endif
			break;
		}
	}

	/*
	 * Set the digipot for the backlight from the schematic
	 *  POT = 0x80, I_BLU = 600mA (MAX do NOT use)
	 *  POT = 0x40, I_BLU = 300mA (mid-init)
	 *  POT = 0x15, I_BLU = 100mA (test)
	 */
	do {
		int ret;
		const int i2cbus = 3; /* MCP4531 I2C BUS 3 */
		const int i2caddr = 0x2e;
		struct udevice *udev;
		uint offset;
		uint val;

		ret = i2c_get_chip_for_busnum(i2cbus, i2caddr, 1, &udev);
		if (ret) {
			printf("Cannot find MCP4531 - error=%x\n", ret);
			break;
		}

		/*
		 * MCP4531 - Address/Command Byte
		 *
		 *          4-bits       2-bits    2-bits
		 *  MSB | Memory Map | Operation | D9/D8 Data | LSB
		 *
		 *  Memory Map:
		 *    Wiper 0 - 0x00
		 *    Wiper 1 - 0x01
		 *    TCON - 0x04
		 *
		 *  Operation:
		 *    Write - 0x0
		 *    Increment - 0x1
		 *    Decrement - 0x2
		 *    Read - 0x3
		 */

		offset = ((0 << 4) | (0 << 2)); /* write wiper0 */
		val = 0x15;
		ret = dm_i2c_reg_write(udev, offset | val >> 8, val & 0xff);
		if (ret) {
			printf("Cannot write %x:%x MCP4531@%x:%x - error=%x\n",
			       offset | val >> 8, val & 0xff,
			       i2cbus, i2caddr, ret);
			break;
		}

		printf("Wrote %x:%x to MCP4531@%x:%x\n",
		       offset | val >> 8, val & 0xff, i2cbus, i2caddr);
	} while(0);

	/*
	 * Set PTN5150A Control Register
	 *
	 * Control 0x2
	 *  [7:5] Reserved
	 *  [4:3] Rp Section (DFP mode)
	 *        00: 80 microA Default
	 *        01: 180 microA Medium
	 *        10: 330 microA High
	 *        11: Reserved
	 *  [2:1] Port Mode Selection
	 *        00: Device (UFP)
	 *        01: Host (DFP)
	 *        10: Dual Role (DRP)
	 *  [0]   Interrupt Mask for detached/attached
	 *        0: Does not Mask Interrupts
	 *        1: Mask Interrupts for register offset 03H bit[1:0].
	 */
#define PTN5150A_BUS	1
#define PTN5150A_ADDR	0x1d
#define PTN5150A_CTRL	0x2
#define PTN5150A_CTRL_RP_MASK   (0x3 << 3)
#define PTN5150A_CTRL_PORT_MASK (0x3 << 1)
	do {
		int ret;
		const int i2cbus = PTN5150A_BUS;
		const int i2caddr = PTN5150A_ADDR;
		uint8_t value;
		struct udevice *udev;

		ret = i2c_get_chip_for_busnum(i2cbus, i2caddr, 1, &udev);
		if (ret) {
			printf("Cannot find PTN5150A - error=%x\n", ret);
			break;
		}

		if (dm_i2c_read(udev, PTN5150A_CTRL, &value, 1)) {
			printf("Cannot read %x:%x PTN5150A@%x:%x - error=%x\n",
			       PTN5150A_CTRL, value,
			       i2cbus, i2caddr, ret);
			break;
		}
		value &= ~(PTN5150A_CTRL_RP_MASK|PTN5150A_CTRL_PORT_MASK);
		value |= (0x1 << 3); /* set to 180 microA */
		value |= (0x2 << 1); /* set to dual role */
		if (dm_i2c_write(udev, PTN5150A_CTRL, &value, 1)) {
			printf("Cannot write %x:%x PTN5150A@%x:%x - error=%x\n",
			       PTN5150A_CTRL, value,
			       i2cbus, i2caddr, ret);
			break;
		}
		printf("Wrote %x:%x to PTN5150A@%x:%x\n",
		       PTN5150A_CTRL, value, PTN5150A_BUS, PTN5150A_ADDR);
	} while(0);

#ifdef CONFIG_MXC_GPIO
	/* activate debug LED 4 to indicate we're about to jump to kernel */
	set_gpio(GPIO_DBG_LED4, "debug_led4", 1);
#endif

	const char *reset_reason = get_reset_reason();
	printf("Reset reason: %s\n", reset_reason);
	bootargs_append_param("resetreason", reset_reason);

	return 0;
}

#if defined(CONFIG_VIDEO_IMXDPUV1)
/*
 * XXX - unclear if ucb will use a display in u-boot, the goal is to fixup
 *       the device tree for linux and avoid changing the bootloader
 */

struct display_info_t const displays[] = {};
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
		uint32_t mac_seed;
		char mac_str[ARP_HLEN_ASCII + 1];

		/* MAC0 */
		imx_get_mac_from_fuse(0, mac_addr);

		debug("imx fuse 0 mac addr %x:%x:%x:%x:%x:%x\n",
			mac_addr[0], mac_addr[1], mac_addr[2],
			mac_addr[3], mac_addr[4], mac_addr[5]);
		if (!is_valid_ethaddr(mac_addr)) {
			mac_seed = uid_low;
			mac_seed ^= mac_seed << 13;
			mac_seed ^= mac_seed >> 17;
			mac_seed ^= mac_seed << 5;
			mac_seed ^= uid_high;
			mac_seed ^= mac_seed << 13;
			mac_seed ^= mac_seed >> 17;
			mac_seed ^= mac_seed << 5;
			mac_addr[0] =
				((mac_seed >> 24) ^ (mac_seed >> 16)) & 0xff;
			mac_addr[1] =
				((mac_seed >> 8) ^ (mac_seed >> 0)) & 0xff;
			mac_seed ^= mac_seed << 13;
			mac_seed ^= mac_seed >> 17;
			mac_seed ^= mac_seed << 5;
			mac_addr[2] = mac_seed >> 24;
			mac_addr[3] = mac_seed >> 16;
			mac_addr[4] = mac_seed >> 8;
			mac_addr[5] = mac_seed >> 0;

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
			mac_seed = uid_high;
			mac_seed ^= mac_seed << 5;
			mac_seed ^= mac_seed >> 17;
			mac_seed ^= mac_seed << 13;
			mac_seed ^= uid_low;
			mac_seed ^= mac_seed << 5;
			mac_seed ^= mac_seed >> 17;
			mac_seed ^= mac_seed << 13;
			mac_addr[0] =
				((mac_seed >> 24) ^ (mac_seed >> 16)) & 0xff;
			mac_addr[1] =
				((mac_seed >> 8) ^ (mac_seed >> 0)) & 0xff;
			mac_seed ^= mac_seed << 5;
			mac_seed ^= mac_seed >> 17;
			mac_seed ^= mac_seed << 13;
			mac_addr[2] = mac_seed >> 24;
			mac_addr[3] = mac_seed >> 16;
			mac_addr[4] = mac_seed >> 8;
			mac_addr[5] = mac_seed >> 0;

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

	/*
	 * Update display reference in the device tree, this just
	 * sets the native-mode reference.
	 */
	do {
		int r;
		const char *display;

		display = env_get("display");
		if (display) {
			r = fdt_fixup_display(blob,
					      fdt_get_alias(blob, "lvds0"),
					      display);
			if (r >= 0) {
				fdt_set_status_by_alias(blob, "lvds0",
							FDT_STATUS_OKAY, 0);
				debug("Enable lvds0 with %s...\n", display);
				break;
			}
		}
	} while(0);

out:
	return 0;
}
#endif
