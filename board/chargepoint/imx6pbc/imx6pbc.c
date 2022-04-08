/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 * Copyright (c) 2019 ChargePoint, Inc.
 *
 * Derived from mx6sabresd reference code with changes specific
 * to this particular board for ChargePoint.
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/arch/sys_proto.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/hab.h>
#include <asm/mach-imx/video.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/pfuze.h"
#include <i2c.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include <mmc.h>
#include <net.h>
#include <fsl_esdhc.h>
#include "../common/bootreason.h"
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

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}


#define I2C_PMIC	2  /* PBC PMIC is on I2C3 */

#ifdef CONFIG_SYS_I2C
#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define I2C_PAD		MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1, I2C_UX_RTC  */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 26)
	}
};

static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 18)
	}
};
#endif /* CONFIG_SYS_I2C */


// Input:
#define PM_INT_N		IMX_GPIO_NR(1,0)
#define SOC_FLOAT_TRIP1		IMX_GPIO_NR(1,4)
#define SOC_FLOAT_TRIP2		IMX_GPIO_NR(1,5)
#define SOC_REED1		IMX_GPIO_NR(7,11)
#define SOC_REED2		IMX_GPIO_NR(7,12)
#define SOC_REED3		IMX_GPIO_NR(7,13)
#define SOC_REED4		IMX_GPIO_NR(4,5)
#define ACCEL_INT_B		IMX_GPIO_NR(6,11)
#define MAG_INT_B		IMX_GPIO_NR(6,14)
#define MAG_DRDY		IMX_GPIO_NR(6,15)
#define SOC_SURGE_DET2		IMX_GPIO_NR(6,7)
#define SOC_SURGE_DET1		IMX_GPIO_NR(6,8)
#define SOC_SURGE_DET3		IMX_GPIO_NR(6,9)
#define SOC_SURGE_DET4		IMX_GPIO_NR(6,10)
#define SOC_SURGE_DET5		IMX_GPIO_NR(2,4)
#define PS2SOC_PG		IMX_GPIO_NR(2,6)
#define FAN_TACH_DSP0		IMX_GPIO_NR(2,14)
#define FAN_TACH_DSP1		IMX_GPIO_NR(2,15)
#define RTD_DRDYn		IMX_GPIO_NR(3,16)
#define SOC_THER_SW1		IMX_GPIO_NR(3,23)
#define SOC_THER_SW2		IMX_GPIO_NR(3,24)
#define SOC_THER_SW3		IMX_GPIO_NR(3,30)
#define SOC_THER_SW4		IMX_GPIO_NR(3,31)
#define PMIC_INT_B		IMX_GPIO_NR(3,29)

// Output:
#define ETH_PWR_EN		IMX_GPIO_NR(1,2)
#define PHY_RSTn		IMX_GPIO_NR(1,3)
#define SER_PWR_EN		IMX_GPIO_NR(1,6)
#define GPIO_DBG_LED5		IMX_GPIO_NR(2,0)
#define GPIO_DBG_LED7		IMX_GPIO_NR(2,1)
#define GPIO_DBG_LED10		IMX_GPIO_NR(2,2)
#define GPIO_DBG_LED9		IMX_GPIO_NR(2,3)
#define FAN1_SP_CTRL		IMX_GPIO_NR(2,12)
#define FAN2_SP_CTRL		IMX_GPIO_NR(2,13)
#define RTD_START_SYNC		IMX_GPIO_NR(3,19)
#define PM_RESET_N		IMX_GPIO_NR(3,20)
#define EN_PMIC_I2C		IMX_GPIO_NR(5,12)
#define GPIO_USBOTG_PWREN	IMX_GPIO_NR(3,22)
#define RS485_DE		IMX_GPIO_NR(3,25)
#define RS485_REn		IMX_GPIO_NR(3,28)

struct gpio_ni {
	const char	*name;
	unsigned	id;
};

static struct gpio_ni gpios_output_low[] = {
#define NI(_x)	{ .name = #_x,  .id = _x }

	NI(GPIO_DBG_LED5),
	NI(GPIO_DBG_LED7),
	NI(GPIO_DBG_LED10),
	NI(GPIO_DBG_LED9),
	NI(FAN1_SP_CTRL),
	NI(FAN2_SP_CTRL),
	NI(RTD_START_SYNC),
	NI(PHY_RSTn),
	NI(RS485_DE),
	NI(RS485_REn),

#undef NI
};

static struct gpio_ni gpios_output_high[] = {
#define NI(_x)	{ .name = #_x,  .id = _x }

	NI(ETH_PWR_EN),
	NI(SER_PWR_EN),
	NI(EN_PMIC_I2C),
	NI(PM_RESET_N),
	NI(GPIO_USBOTG_PWREN),

#undef NI
};

static const struct gpio_ni gpios_input[] = {
#define NI(_x)	{ .name = #_x,  .id = _x }

	NI(PM_INT_N),
	NI(SOC_FLOAT_TRIP1),
	NI(SOC_FLOAT_TRIP2),
	NI(SOC_REED1),
	NI(SOC_REED2),
	NI(SOC_REED3),
	NI(SOC_REED4),
	NI(ACCEL_INT_B),
	NI(MAG_INT_B),
	NI(MAG_DRDY),
	NI(SOC_SURGE_DET2),
	NI(SOC_SURGE_DET1),
	NI(SOC_SURGE_DET3),
	NI(SOC_SURGE_DET4),
	NI(SOC_SURGE_DET5),
	NI(PS2SOC_PG),
	NI(FAN_TACH_DSP0),
	NI(FAN_TACH_DSP1),
	NI(RTD_DRDYn),
	NI(SOC_THER_SW1),
	NI(SOC_THER_SW2),
	NI(SOC_THER_SW3),
	NI(SOC_THER_SW4),
	NI(PMIC_INT_B),

#undef NI
};

static void set_gpios_input(const struct gpio_ni *nip, unsigned int cnt)
{
	unsigned int i;

	debug("%s >>>>>>>>\n", __func__);
	for (i = 0; i < cnt; i++) {
		gpio_request(nip[i].id, nip[i].name);
		gpio_direction_input(nip[i].id);
		debug("\t%d: %s[%u] <- (input)\n", i, nip[i].name, nip[i].id);
	}
}

static void set_gpios(const struct gpio_ni *nip, unsigned int cnt, int val)
{
	unsigned int i;

	debug("%s >>>>>>>>\n", __func__);
	for (i = 0; i < cnt; i++) {
		gpio_request(nip[i].id, nip[i].name);
		gpio_direction_output(nip[i].id, val);
		debug("\t%d: %s[%u] -> %#x\n", i, nip[i].name, nip[i].id, val);
	}
}

static void set_gpio(int gpio, const char *gpioname, int val)
{
	debug("%s >>>>>>>>\n", __func__);
	gpio_request(gpio, gpioname);
	gpio_direction_output(gpio, val);
	debug("\t%s[%u] -> %#x\n", gpioname, gpio, val);
}

#if defined(CONFIG_FSL_ESDHC)
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC1_BASE_ADDR},
};

int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno + 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC3 is always present */
		break;
	default:
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    eEMMC/uSDHC3
	 * mmc1                    SD1/uSDHC1
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			usdhc_cfg[0].max_bus_width = 8; /* eMMC bus width */
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 1:
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}
#endif /* CONFIG_FSL_ESDHC */

#if defined(CONFIG_VIDEO_IPUV3)
/* XXX - unclear if pbc will get a display, so stub out for now */
struct display_info_t const displays[] = {};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	return;
}
#endif /* CONFIG_VIDEO_IPUV3 */


/* forward declare pin_mux init since pin_mux.h cannot be included */
extern void BOARD_InitPins(void);

int board_early_init_f(void)
{
	/* call generated pin_mux code from the NXP pin tools */
	BOARD_InitPins();

	/* select 1P5V drive strength on RGMII i/o pins */
	__raw_writel(DDR_SEL_1P5V_IO,
		     (void *)IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII);

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

#if defined(CONFIG_SECURE_BOOT)
	if (imx_hab_is_enabled()) {
		gd->flags |= (GD_FLG_SILENT | GD_FLG_DISABLE_CONSOLE);
	}
#endif

	return 0;
}

int board_init(void)
{

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	/* Device specific GPIO settings */
	set_gpios_input(gpios_input, ARRAY_SIZE(gpios_input));
	set_gpios(gpios_output_high, ARRAY_SIZE(gpios_output_high), 1);
	set_gpios(gpios_output_low, ARRAY_SIZE(gpios_output_low), 0);

#ifdef CONFIG_SYS_I2C
	/* Enable access to PMIC on I2C */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif

#ifdef CONFIG_USB_EHCI_MX6
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 *
	 * OTG ID daisy chain set
	 * 0: ENET_RX_ER as USB_OTG_ID
	 * 1: GPIO_1 as USB_OTG_ID
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 1);
#endif

	setup_fitimage_keys();

	return 0;
}

#ifdef CONFIG_POWER
int power_init_board(void)
{
	struct pmic *pfuze;
	unsigned int reg;
	int ret;

	pfuze = pfuze_common_init(I2C_PMIC);
	if (!pfuze)
		return -ENODEV;

	if (is_mx6dqp())
		ret = pfuze_mode_init(pfuze, APS_APS);
	else
		ret = pfuze_mode_init(pfuze, APS_PFM);
	if (ret < 0)
		return ret;

	/* VGEN3 and VGEN5 corrected on i.mx6qp board */
	if (!is_mx6dqp()) {
		/* Increase VGEN3 from 2.5 to 2.8V */
		pmic_reg_read(pfuze, PFUZE100_VGEN3VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_2_80V;
		pmic_reg_write(pfuze, PFUZE100_VGEN3VOL, reg);

		/* Increase VGEN5 from 2.8 to 3V */
		pmic_reg_read(pfuze, PFUZE100_VGEN5VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_3_00V;
		pmic_reg_write(pfuze, PFUZE100_VGEN5VOL, reg);
	}
	if (is_mx6dqp()) {
		/* set SW1C staby volatage 1.075V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1f;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);

		/* set SW2/VDDARM staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW2STBY, &reg);
		reg &= ~0x3f;
		reg |= 0x17;
		pmic_reg_write(pfuze, PFUZE100_SW2STBY, reg);

		/* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW2CONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW2CONF, reg);
	} else {
		/* set SW1AB staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1ABSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1ABSTBY, reg);

		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1ABCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1ABCONF, reg);

		/* set SW1C staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);
	}

	return 0;
}
#elif defined(CONFIG_DM_PMIC_PFUZE100)
int power_init_board(void)
{
	struct udevice *dev;
	unsigned int reg;
	int ret;

	dev = pfuze_common_init();
	if (!dev)
		return -ENODEV;

	if (is_mx6dqp())
		ret = pfuze_mode_init(dev, APS_APS);
	else
		ret = pfuze_mode_init(dev, APS_PFM);
	if (ret < 0)
		return ret;

	/* VGEN3 and VGEN5 corrected on i.mx6qp board */
	if (!is_mx6dqp()) {
		/* Increase VGEN3 from 2.5 to 2.8V */
		reg = pmic_reg_read(dev, PFUZE100_VGEN3VOL);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_2_80V;
		pmic_reg_write(dev, PFUZE100_VGEN3VOL, reg);

		/* Increase VGEN5 from 2.8 to 3V */
		reg = pmic_reg_read(dev, PFUZE100_VGEN5VOL);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_3_00V;
		pmic_reg_write(dev, PFUZE100_VGEN5VOL, reg);
	}
	if (is_mx6dqp()) {
		/* set SW1C staby volatage 1.075V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
		reg &= ~0x3f;
		reg |= 0x1f;
		pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);

		/* set SW2/VDDARM staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW2STBY);
		reg &= ~0x3f;
		reg |= 0x17;
		pmic_reg_write(dev, PFUZE100_SW2STBY, reg);

		/* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW2CONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW2CONF, reg);
	} else {
		/* set SW1AB staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1ABSTBY);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(dev, PFUZE100_SW1ABSTBY, reg);

		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1ABCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1ABCONF, reg);

		/* set SW1C staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);
	}

	return 0;
}
#endif

#ifdef CONFIG_LDO_BYPASS_CHECK
#ifdef CONFIG_POWER
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	struct pmic *p = pmic_get("PFUZE100");

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0;	/* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x29;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
			value |= 0x2d;
			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.425V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x2d;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);
	}

	return;
}
#elif defined(CONFIG_DM_PMIC_PFUZE100)
void ldo_mode_set(int ldo_bypass)
{
	struct udevice *dev;
	int ret;

	ret = pmic_get("pfuze100", &dev);
	if (ret == -ENODEV) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0; /* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x29);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x2d);
		}
		/* increase VDDSOC to 1.425V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x2d);
	}

	return;
}
#endif
#endif

int board_late_init(void)
{

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
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

#if defined(CONFIG_SECURE_BOOT)
	/* set an environment that this is a secure boot */
	if (imx_hab_is_enabled()) {
		env_set("bootargs_secureboot", "uboot-secureboot");
	}
#endif

	/* activate debug LED 5 to indicate we're about to jump to kernel */
	set_gpio(GPIO_DBG_LED5, "dbg_led_5", 1);

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
#ifdef CONFIG_DM_USB
int board_ehci_hcd_init(int port)
{

	switch (port) {
	case 0:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_DM_USB */
#endif

int ft_board_setup(void *blob, bd_t *bd)
{
	uint32_t uid_high, uid_low;

	/*
	 * i.MX6 Configuration and Manufacturing Info
	 *   OCOTP_CFG0: OTP Bank 0, word 1
	 *   OCOTP_CFG1: OTP Bank 0, word 2
	 */
	do {
		struct ocotp_regs *ocotp =
			(struct ocotp_regs *)OCOTP_BASE_ADDR;
		struct fuse_bank *bank = &ocotp->bank[0];
		struct fuse_bank0_regs *fuse =
			(struct fuse_bank0_regs *)bank->fuse_regs;

		/* Make this a valid MAC address and set it */
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

	if (!env_get("ethaddr")) {
		u8 mac_addr[6];
		uint32_t mac_seed;
		char mac_str[ARP_HLEN_ASCII + 1];
		uint32_t mac_val1, mac_val0;
		struct ocotp_regs *ocotp =
			(struct ocotp_regs *)OCOTP_BASE_ADDR;
		struct fuse_bank *bank = &ocotp->bank[4];
		struct fuse_bank4_regs *fuse =
			(struct fuse_bank4_regs *)bank->fuse_regs;

		mac_val1 = readl(&fuse->mac_addr1);
		mac_val0 = readl(&fuse->mac_addr0);
		mac_addr[0] = mac_val1 >> 8;
		mac_addr[1] = mac_val1 >> 0;
		mac_addr[2] = mac_val0 >> 24;
		mac_addr[3] = mac_val0 >> 16;
		mac_addr[4] = mac_val0 >> 8;
		mac_addr[5] = mac_val0 >> 0;

		debug("imx fuse mac addr %x:%x:%x:%x:%x:%x\n",
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

		fdt_fixup_ethernet(blob);
	}

	return 0;
}
