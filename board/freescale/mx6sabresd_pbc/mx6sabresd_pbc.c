/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/spi.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>

#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <ipu_pixfmt.h>
#include <linux/fb.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <asm/arch/mx6-ddr.h>
#include <usb.h>
#include <environment.h>

void BOARD_InitPins(void);

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PMIC	2  /* PBC PMIC is on I2C3 */

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
    PAD_CTL_DSE_40ohm | PAD_CTL_HYS |           \
    PAD_CTL_ODE)

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}


// Input:
#define PM_INT_N            IMX_GPIO_NR(1,0)
#define SOC_FLOAT_TRIP1     IMX_GPIO_NR(1,4)
#define SOC_FLOAT_TRIP2     IMX_GPIO_NR(1,5)
#define SOC_REED1           IMX_GPIO_NR(7,11)
#define SOC_REED2           IMX_GPIO_NR(7,12)
#define SOC_REED3           IMX_GPIO_NR(7,13)
#define SOC_REED4           IMX_GPIO_NR(4,5)
#define ACCEL_INT_B         IMX_GPIO_NR(6,11)
#define MAG_INT_B           IMX_GPIO_NR(6,14)
#define MAG_DRDY            IMX_GPIO_NR(6,15)
#define SOC_SURGE_DET2      IMX_GPIO_NR(6,7)
#define SOC_SURGE_DET1      IMX_GPIO_NR(6,8)
#define SOC_SURGE_DET3      IMX_GPIO_NR(6,9)
#define SOC_SURGE_DET4      IMX_GPIO_NR(6,10)
#define SOC_SURGE_DET5      IMX_GPIO_NR(2,4)
#define PS2SOC_PG           IMX_GPIO_NR(2,6)
#define FAN_TACH_DSP0       IMX_GPIO_NR(2,14)
#define FAN_TACH_DSP1       IMX_GPIO_NR(2,15)
#define RTD_DRDYn           IMX_GPIO_NR(3,16)
#define SOC_THER_SW1        IMX_GPIO_NR(3,23)
#define SOC_THER_SW2        IMX_GPIO_NR(3,24)
#define SOC_THER_SW3        IMX_GPIO_NR(3,30)
#define SOC_THER_SW4        IMX_GPIO_NR(3,31)
#define PMIC_INT_B          IMX_GPIO_NR(3,29)
#define USB_OTG_ID          IMX_GPIO_NR(1,1)

// Output:
#define ETH_PWR_EN          IMX_GPIO_NR(1,2)
#define PHY_RSTn            IMX_GPIO_NR(1,3)
#define SER_PWR_EN          IMX_GPIO_NR(1,6)
#define GPIO2_0             IMX_GPIO_NR(2,0)
#define GPIO2_1             IMX_GPIO_NR(2,1)
#define GPIO2_2             IMX_GPIO_NR(2,2)
#define GPIO2_3             IMX_GPIO_NR(2,3)
#define FAN1_SP_CTRL        IMX_GPIO_NR(2,12)
#define FAN2_SP_CTRL        IMX_GPIO_NR(2,13)
#define RTD_START_SYNC      IMX_GPIO_NR(3,19)
#define PM_RESET_N          IMX_GPIO_NR(3,20)
#define EN_PMIC_I2C         IMX_GPIO_NR(5,12)
#define RS485_DE            IMX_GPIO_NR(3,25)
#define RS485_REn           IMX_GPIO_NR(3,28)
#define USB_OTG_PWR         IMX_GPIO_NR(3,22)


#define GPIO_DBG_LED5       GPIO2_0
#define GPIO_DBG_LED7       GPIO2_1
#define GPIO_DBG_LED10      GPIO2_2
#define GPIO_DBG_LED9       GPIO2_3


static void setup_iomux_enet(void)
{
	/* Reset LAN PHY */
	gpio_direction_output(PHY_RSTn , 0);
	udelay(500); /* VG : Need to tune eth startup delay */ 
	gpio_set_value(PHY_RSTn, 1);
}

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

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC1_BASE_ADDR},
};

int mmc_get_env_devno(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 dev_no;
	u32 bootsel;

	bootsel = (soc_sbmr & 0x000000FF) >> 6 ;
#ifdef CONFIG_TARGET_MX6DLSABRESD_PBC
        printf("mmc_get_env_devno(): CONFIG_SYS_MMC_ENV_DEV...\n");
        return CONFIG_SYS_MMC_ENV_DEV;
#else
        printf("mmc_get_env_devno(): NOT CONFIG_SYS_MMC_ENV_DEV...\n");
#endif
	/* If not boot from sd/mmc, use default value */
	if (bootsel != 1)
		return CONFIG_SYS_MMC_ENV_DEV;

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	/* need ubstract 1 to map to the mmc device id
	 * see the comments in board_mmc_init function
	 */

	dev_no--;

	return dev_no;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no + 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

    switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC3 is always present on PBC board*/
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	int i;

//board_eth_init(bis);
//printf("setup_iomux_enet(): ...\n");
//setup_iomux_enet();

    printf("board_mmc_init(): ...\n");
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


int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

#if 1
void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_devno();

    if (!check_mmc_autodetect())
		return;

	env_set_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
        /* sprintf(mmcblk, "/dev/mmcblk%dp5 rootwait rw",
		mmc_map_to_kernel_blk(dev_no)); */
	sprintf(mmcblk, "/dev/mmcblk%dp5 rootwait rw",dev_no);
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}
#endif

struct display_info_t const displays[] = {};

size_t display_count = ARRAY_SIZE(displays);

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
    printf("board_eth_init(): ...\n");
	setup_iomux_enet();
	return cpu_eth_init(bis);
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static void setup_usb(void)
{
    printf("setup_usb(): ...\n");
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 *
         * OTG ID daisy chain set
         * 0: ENET_RX_ER as USB_OTG_ID
         * 1: GPIO_1 as USB_OTG_ID
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 1);
}

int board_ehci_hcd_init(int port)
{
        u32 *usbnc_usb_ctrl;

        printf("board_ehci_hcd_init(): ...\n");
       if (port > 1)
                return -EINVAL;

        usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
                                 port * 4);

        setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
    printf("board_ehci_power(): ...\n");

    switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(USB_OTG_PWR, 1);
		else
			gpio_direction_output(USB_OTG_PWR, 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif
static unsigned gpios_output_low[] = {
        GPIO2_0,
        GPIO2_1,
        GPIO2_2,
        GPIO2_3,
        FAN1_SP_CTRL,
        FAN2_SP_CTRL,
        RTD_START_SYNC,
        RS485_DE,
        RS485_REn
};

static const char *gpios_output_low_names[] = {
        "GPIO2_0",
        "GPIO2_1",
        "GPIO2_2",
        "GPIO2_3",
        "FAN1_SP_CTRL",
        "FAN2_SP_CTRL",
        "RTD_START_SYNC",
        "RS485_DE",
        "RS485_REn"
};

static unsigned gpios_output_high[] = {
        ETH_PWR_EN,
        PHY_RSTn,
        SER_PWR_EN,
        EN_PMIC_I2C,
        USB_OTG_PWR,
        PM_RESET_N
};

static const char *gpios_output_high_names[] = {
        "ETH_PWR_EN",
        "PHY_RSTn",
        "SER_PWR_EN",
        "EN_PMIC_I2C",
        "USB_OTG_PWR",
        "PM_RESET_N"
};

static const unsigned gpios_input[] = {
        PM_INT_N,
        SOC_FLOAT_TRIP1,
        SOC_FLOAT_TRIP2,
        SOC_REED1,
        SOC_REED2,
        SOC_REED3,
        SOC_REED4,
        ACCEL_INT_B,
        MAG_INT_B,
        MAG_DRDY,
        SOC_SURGE_DET2,
        SOC_SURGE_DET1,
        SOC_SURGE_DET3,
        SOC_SURGE_DET4,
        SOC_SURGE_DET5,
        PS2SOC_PG,
        FAN_TACH_DSP0,
        FAN_TACH_DSP1,
        RTD_DRDYn,
        SOC_THER_SW1,
        SOC_THER_SW2,
        SOC_THER_SW3,
        SOC_THER_SW4,
        PMIC_INT_B,
        USB_OTG_ID
};

static const char *gpios_input_names[] = {
        "PM_INT_N",
        "SOC_FLOAT_TRIP1",
        "SOC_FLOAT_TRIP2",
        "SOC_REED1",
        "SOC_REED2",
        "SOC_REED3",
        "SOC_REED4",
        "ACCEL_INT_B",
        "MAG_INT_B",
        "MAG_DRDY",
        "SOC_SURGE_DET2",
        "SOC_SURGE_DET1",
        "SOC_SURGE_DET3",
        "SOC_SURGE_DET4",
        "SOC_SURGE_DET5",
        "PS2SOC_PG",
        "FAN_TACH_DSP0",
        "FAN_TACH_DSP1",
        "RTD_DRDYn",
        "SOC_THER_SW1",
        "SOC_THER_SW2",
        "SOC_THER_SW3",
        "SOC_THER_SW4",
        "PMIC_INT_B",
        "USB_OTG_ID"
};

static void gpios_request(const unsigned *p, const char **pname, int cnt)
{
        int i;

        for (i = 0; i < cnt; i++)
            gpio_request(*p++, *pname++);
}

static void set_gpios_input(const unsigned *p, int cnt)
{
        int i;

        for (i = 0; i < cnt; i++)
                gpio_direction_input(*p++);
}

static void set_gpios(unsigned *p, int cnt, int val)
{
        int i;

        for (i = 0; i < cnt; i++)
                gpio_direction_output(*p++, val);
}

int board_early_init_f(void)
{
    BOARD_InitPins();
    // select 1P5V drive strength on RGMII i/o pins
    __raw_writel(DDR_SEL_1P5V_IO, (void *)IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII);

	return 0;
}

int board_init(void)
{
    printf("board_early_init_f():  BOARD_InitPins()...\n");

    /* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

printf("board_init(): 0...\n");

    /* Device specific GPIO settings */
    gpios_request(gpios_input, gpios_input_names, ARRAY_SIZE(gpios_input));
    gpios_request(gpios_output_high, gpios_output_high_names, ARRAY_SIZE(gpios_output_high));
    gpios_request(gpios_output_low, gpios_output_low_names, ARRAY_SIZE(gpios_output_low));

//printf("board_init(): 1...\n");

    set_gpios_input(gpios_input, ARRAY_SIZE(gpios_input));
    set_gpios(gpios_output_high, ARRAY_SIZE(gpios_output_high), 1);
    set_gpios(gpios_output_low, ARRAY_SIZE(gpios_output_low), 0);

//printf("board_init(): 2...\n");

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	gpio_direction_output(EN_PMIC_I2C, 1);

printf("board_init(): 3...\n");

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

static struct pmic *pfuze;
int power_init_board(void)
{
    printf("power_init_board(): ...\n");
	/*unsigned int reg, ret;*/
	pfuze = pfuze_common_init(I2C_PMIC);
	/*if (!pfuze)
		return -ENODEV;*/
#if 0
	ret = pfuze_mode_init(pfuze, APS_PFM);
	if (ret < 0)
		return ret;

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
#endif
	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	int is_400M;
	unsigned char vddarm;
	struct pmic *p = pfuze;

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
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();
		if (is_mx6dqp()) {
			/* decrease VDDARM for 400Mhz DQP:1.1V*/
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x1c;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
#if defined(CONFIG_MX6DL)
			value |= 0x27;
#else
			value |= 0x20;
#endif

			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.3V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x28;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		/*
		 * MX6Q/DQP:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.075V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_mx6dqp()) {
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			if (is_400M)
				value |= 0x17;
			else
				value |= 0x1e;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		}

		if (is_400M)
#if defined(CONFIG_MX6DL)
			vddarm = 0x1f;
#else
			vddarm = 0x1b;
#endif
		else
#if defined(CONFIG_MX6DL)
			vddarm = 0x23;
#else
			vddarm = 0x22;
#endif
		pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
		value &= ~0x3f;
		value |= vddarm;
		pmic_reg_write(p, PFUZE100_SW1ABVOL, value);

		/* decrease VDDSOC to 1.175V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x23;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
        char *env = NULL;
        char *btype = "unknown";

        printf("board_late_init(): ...\n");

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	gpio_direction_output(GPIO_DBG_LED7, 1);

        if (is_cpu_type(MXC_CPU_MX6DP)) { btype = "pbc"; }
        env = env_get("board_type");
        if(!env || strcmp(env, btype)){
               set_default_env(NULL);
               env_set("board_type", btype);
               printf("Saving env in flash \n");
               env_save();
        }
        printf("Reset Cause 0x%08X \n",get_imx_reset_cause());
        env_set_ulong("reset_cause", get_imx_reset_cause());
	return 0;
}

int checkboard(void)
{
	puts("Board: ChargePoint PBC\n");
	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!env_get("fastboot_dev"))
			env_set("fastboot_dev", "sata");
		if (!env_get("bootcmd"))
			env_set("bootcmd", "boota sata");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
	    if (!env_get("fastboot_dev"))
			env_set("fastboot_dev", "mmc0");
	    if (!env_get("bootcmd"))
			env_set("bootcmd", "boota mmc0");
	    break;
	case SD3_BOOT:
	case MMC3_BOOT:
	    if (!env_get("fastboot_dev"))
			env_set("fastboot_dev", "mmc1");
	    if (!env_get("bootcmd"))
			env_set("bootcmd", "boota mmc1");
	    break;
	case MMC4_BOOT:
	    if (!env_get("fastboot_dev"))
			env_set("fastboot_dev", "mmc2");
	    if (!env_get("bootcmd"))
			env_set("bootcmd", "boota mmc2");
	    break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}

}
#endif /*CONFIG_FSL_FASTBOOT*/

