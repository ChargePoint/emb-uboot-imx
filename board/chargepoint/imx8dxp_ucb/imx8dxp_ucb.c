// SPDX-License-Identifier: GPL-2.0+
/*
 * ChargePoint UCB U-Boot
 *
 * Portions Copyright 2018 NXP
 */

#include <common.h>
#include <cpu_func.h>
#include <errno.h>
#include <init.h>
#include <fdt_support.h>
#include <linux/libfdt.h>
#include <env.h>
#include <i2c.h>
#include <fsl_esdhc_imx.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/snvs_security_sc.h>
#include <dm.h>
#include <usb.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/imx8qxp_lpcg.h>
#include <asm/arch/lpcg.h>

#if CONFIG_IS_ENABLED(FEC_MXC)
#include <miiphy.h>
#endif

#include "../common/bootreason.h"
#include "../common/fitimage_keys.h"

DECLARE_GLOBAL_DATA_PTR;


enum {
	UCB_BOARDID1 = 0x1, /* UCBv4 to UCBv6 */
	UCB_BOARDID2 = 0x2, /* UCBv7 and up */
};

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

#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | \
			 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | \
			 (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

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

	imx8_iomux_setup_multiple_pads(uart0_pads, ARRAY_SIZE(uart0_pads));

	return 0;
}

#if CONFIG_IS_ENABLED(DM_GPIO)
static int dm_get_gpio(const char *name, const char *label)
{
	struct gpio_desc desc;
	int ret;

	ret = dm_gpio_lookup_name(name, &desc);
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

	ret = dm_gpio_get_value(&desc);
	dm_gpio_free(desc.dev, &desc);
	return ret;
}

static void dm_set_gpio(const char *name, const char *label, int val)
{
	struct gpio_desc desc;
	int ret;

	ret = dm_gpio_lookup_name(name, &desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, name, ret);
		return;
	}

	ret = dm_gpio_request(&desc, label);
	if (ret) {
		printf("%s dm_gpio_request for %s failed ret = %d\n",
		       __func__, label, ret);
		return;
	}

	ret = dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	if (ret) {
		printf("%s dm_gpio_set_dir_flags for %s failed ret = %d\n",
		       __func__, label, ret);
		goto out;
	}

	ret = dm_gpio_set_value(&desc, val);
	if (ret) {
		printf("%s dm_gpio_set_value for %s failed ret = %d\n",
		       __func__, label, ret);
		goto out;
	}

out:
	dm_gpio_free(desc.dev, &desc);
	return;
}

static void board_gpio_init(void)
{
	dm_set_gpio("gpio1_8", "debug_led4", 0);
	dm_set_gpio("gpio1_7", "debug_led5", 0);
	dm_set_gpio("gpio3_14", "debug_led11", 0);

	dm_set_gpio("gpio1_6", "usb5734_reset", 0);

	dm_set_gpio("gpio0_29", "ser_pwr_en", 1);
}

static int get_boardid(void)
{
	return ((dm_get_gpio("gpio4_5", "bd_id_1") << 1) |
		(dm_get_gpio("gpio4_3", "bd_id_0") << 0));
}
#endif // IS_ENABLED(DM_GPIO)

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	return ret;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	return ret;
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

		/* HIFI DSP boot */
		"audio_sai0",
		"audio_ocram",
	};

	imx8_power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(void)
{
	sc_pm_reboot(-1, SC_PM_RESET_TYPE_COLD);
	while(1);

}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int board_late_init(void)
{
	sc_err_t err;
	uint16_t lc;
	sc_ipc_t ipcHndl = -1;

	build_info();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "UCB");
	env_set("board_rev", "iMX8DXP");

	switch (get_boardid()) {
	case UCB_BOARDID1:
		env_set("boardid_name", "UCB_BOARDID1");
		break;
	default:
	case UCB_BOARDID2:
		env_set("boardid_name", "UCB_BOARDID2");
		break;
	}
#endif

#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#else
	env_set("sec_boot", "no");
#endif

	/*
	 * Set the reason the system was reset by reading the watchdog
	 * reset reason.
	 */
	int appendcnt = 0;
	char appendargs[512] = {0};
	appendcnt += scnprintf(&appendargs[appendcnt],
			       sizeof(appendargs) - appendcnt,
			       " resetreason=%s", get_wdog_reset_reason());
	env_set("bootargs_append", appendargs);

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
			/* set an environment that this is a secure boot */
			env_set("bootargs_secureboot", "uboot-secureboot");
			break;
		}
	} else {
		printf("%s: sc_seco_chip_info error %d\n", __func__, err);
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

#if CONFIG_IS_ENABLED(DM_GPIO)
	/* activate debug LED 4 to indicate we're about to jump to kernel */
	dm_set_gpio("gpio1_8", "debug_led4", 1);
#endif

	return 0;
}

#if CONFIG_IS_ENABLED(FEC_MXC)
static enum phy_vtype_e {
	PHY_VENDOR_QUALCOMM = 1,
	PHY_VENDOR_REALTEK,
	PHY_VENDOR_MAX
} phyType;

int board_phy_config(struct phy_device *phydev)
{
	unsigned short id1, id2;

	/*
	 * assume the phy vendor is qualcomm - this can be verified by
	 * the phyid as needed
	 */
	phyType = PHY_VENDOR_QUALCOMM;

	/* check for realtek phy id based on datasheet */
	id1 = phy_read(phydev, MDIO_DEVAD_NONE, 2);
	id2 = phy_read(phydev, MDIO_DEVAD_NONE, 3);
	if ((id1 == 0x001c) && (id2 == 0xc916)) {
		phyType = PHY_VENDOR_REALTEK;
	}

	return 0;
}
#endif /* CONFIG_IS_ENABLED(FEC_MXC) */

int checkboard(void)
{
	puts("Board: iMX8DXP UCB\n");

	print_bootinfo();

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
struct codec {
	const char *name;
	const char *i2c_path;
	const char *sound_path;
};

/*
 * codec_list index is tied to board id. Any alteration to
 * this list will cause issue, the order needs to maintained.
 * If new codec added to the list it must be associated with
 * a boardid.
 */
static const struct codec codec_list[] = {
	{"wm8974", "/bus@5a000000/i2c@5a810000/wm8974@1a/", "/sound-wm8974"},
	{"wm8940", "/bus@5a000000/i2c@5a810000/wm8940@1a/", "/sound-wm8940"}
};

static void set_audio_codec(void *blob)
{
	int cid;
	int offs;

	switch (get_boardid()) {
	case UCB_BOARDID1:
		cid = 0;
		break;
	default:
	case UCB_BOARDID2:
		cid = 1;
		break;
	}

	printf("Audio Codec: %s\n", codec_list[cid].name);

	offs = fdt_path_offset(blob, codec_list[cid].i2c_path);
	if (fdt_setprop_string(blob, offs, "status", "okay") < 0) {
		printf("  Failed to set %s/status -> okay\n",
		       codec_list[cid].i2c_path);
	}
	offs = fdt_path_offset(blob, codec_list[cid].sound_path);
	if (fdt_setprop_string(blob, offs, "status", "okay") < 0) {
		printf("  Failed to set %s/status -> okay\n",
		       codec_list[cid].sound_path);
	}
}

/* update device tree to support realtek specific parameters */
#define ETHPHY0_PATH "/bus@5b000000/ethernet@5b040000"
#define ETHPHY1_PATH "/bus@5b000000/ethernet@5b050000"
#define MDIO_ETH_PATH "/mdio/ethernet-phy@/"
#define ETHPHY0_MDIO_PATH "/bus@5b000000/ethernet@5b040000/mdio/ethernet-phy@0/"
#define ETHPHY1_MDIO_PATH "/bus@5b000000/ethernet@5b040000/mdio/ethernet-phy@1/"

static void realtek_phy_supp(void *blob)
{
	int offs;

	offs = fdt_path_offset(blob,ETHPHY0_PATH);
	if (fdt_setprop_string(blob, offs, "phy-mode","rgmii-id") < 0) {
		printf("fdt eth0 phy-mode FDT_ERR_NOTFOUND\n");
	}
	if (fdt_delprop(blob, offs, "fsl,rgmii_rxc_dly") < 0) {
		printf("fdt eth0 rgmii_rxc_dly FDT_ERR_NOTFOUND\n");
	}
	if (fdt_delprop(blob, offs, "fsl,ar8031-phy-fixup") < 0) {
		printf("fdt eth0 ar8031-phy-fixup FDT_ERR_NOTFOUND\n");
	}

	offs = fdt_path_offset(blob,ETHPHY1_PATH);
	if (fdt_setprop_string(blob, offs, "phy-mode","rgmii-id") < 0) {
		printf("fdt eth1 phy-mode FDT_ERR_NOTFOUND\n");
	}
	if (fdt_delprop(blob, offs, "fsl,rgmii_rxc_dly") < 0) {
		printf("fdt eth1 rgmii_rxc_dly FDT_ERR_NOTFOUND\n");
	}
	if (fdt_delprop(blob, offs, "fsl,ar8031-phy-fixup") < 0) {
		printf("fdt eth1 ar8031-phy-fixup FDT_ERR_NOTFOUND\n");
	}

	offs = fdt_path_offset(blob,ETHPHY0_MDIO_PATH);
	if (fdt_setprop_u32(blob, offs, "reg",1) < 0) {
		printf("fdt eth0 reg FDT_ERR_NOTFOUND\n");
	}

	offs = fdt_path_offset(blob,ETHPHY1_MDIO_PATH);
	if (fdt_setprop_u32(blob, offs, "reg",2) < 0) {
		printf("fdt eth0 reg FDT_ERR_NOTFOUND\n");
	}
	return;
}

int ft_board_setup(void *blob, struct bd_info *bd)
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

		ipcHndl = -1; // was gd->arch.ipc_channel_handle;

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

	/* TODO - replace with a dtb overlay based on boardid */
	set_audio_codec(blob);

	printf("Phy vendor: %d\n", phyType);
	if (phyType != PHY_VENDOR_QUALCOMM) {
		realtek_phy_supp(blob);
	}

	/*
	 * incorporate the core of a pseudo random generator from xorshift32
	 * that is derived from p. 4 of Marsaglia, "Xorshift RNGs"
	 *
	 * uint32_t xorshift32(struct xorshift32_state *state)
	 * {
	 *         uint32_t x = state->a;
	 *         x ^= x << 13;
	 *         x ^= x >> 17;
	 *         x ^= x << 5;
	 *         return state->a = x;
	 * }
	 *
	 * The collision of 64-bit to 46-bit because of multicast and locally
	 * derived bits is a concern, but the cpuid values are generated from
	 * NXP as a consecutive sequence meaning a run from the same parts
	 * lot may only differ in 1-bit which is a larger concern then the
	 * collision domains. Because the seed is not really a random value
	 * it is more important to use a PRNG generator to deal with small
	 * bit differences instead of just hashing the cpuid for a value.
	 */
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
							FDT_STATUS_OKAY);
				debug("Enable lvds0 with %s...\n", display);
				break;
			}
		}
	} while(0);

out:
	return 0;
}
#endif
