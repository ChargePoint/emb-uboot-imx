/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
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

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define CSI_PAD_CTRL    (PAD_CTL_PUS_100K_UP |                  \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |                 \
        PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define AUD_PAD_CTRL  (PAD_CTL_PUS_100K_UP |                    \
        PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |                 \
        PAD_CTL_HYS | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define WEAK_PULLDN     (PAD_CTL_PUS_100K_DOWN |                \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |                 \
        PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define I2C_PMIC	2  /* CPNK PMIC is on I2C3 */

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const uart2_rs485_pads[] = {
        MX6_PAD_EIM_D26__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_EIM_D27__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	/* RS485 control GPIO */
#define GPIO_RS485_TX_EN          IMX_GPIO_NR(3, 25)
	MX6_PAD_EIM_D25__GPIO3_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define GPIO_RS485_RX_EN          IMX_GPIO_NR(3, 28)
        MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__SD1_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__SD1_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__SD1_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__SD1_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__SD1_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__SD1_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_RST__SD3_RESET | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__ENET_REF_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* Ethernet PHY Reset */
	MX6_PAD_ENET_REF_CLK__GPIO1_IO23	| MUX_PAD_CTRL(NO_PAD_CTRL),
#define GPIO_ETH_PHY_RESET IMX_GPIO_NR(1, 23)
	/* CPU_ENET_INTn*/
	MX6_PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* CPU Remote Reset */
	MX6_PAD_KEY_ROW4__GPIO4_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL), /* UX_5V_OC */
};

static iomux_v3_cfg_t const cpnk_init_pads[] = {
        /* Last GASP controls */
#define GPIO_LG_CHGEN IMX_GPIO_NR(1, 13)
	MX6_PAD_SD2_DAT2__GPIO1_IO13  | MUX_PAD_CTRL(NO_PAD_CTRL), /* LG CHG_ENABLE */
#define GPIO_LG_CAPGOOD IMX_GPIO_NR(1, 15)
	MX6_PAD_SD2_DAT0__GPIO1_IO15  | MUX_PAD_CTRL(NO_PAD_CTRL), /* LG CHG_CAPGOOD */
#define GPIO_LG_CHGPFW IMX_GPIO_NR(1, 2)
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), /* LG CHG_PFWn */
        /* PMIC Interrupt */
#define GPIO_PMIC_INT IMX_GPIO_NR(3, 29)
        MX6_PAD_EIM_D29__GPIO3_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL), /* PMIC_INT_B */
#define GPIO_PMIC_EN IMX_GPIO_NR(5, 12)
        MX6_PAD_DISP0_DAT18__GPIO5_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), /* GPIO_PMIC_EN */
        /* WIF module power control */
#define GPIO_WIF_PWREN IMX_GPIO_NR(3, 19)
	MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WIF_PWREN */
#define GPIO_WIF_RESET IMX_GPIO_NR(3, 20)
	MX6_PAD_EIM_D20__GPIO3_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL), /* WIF_RESETn */
        /* DBG LED Control */
#define GPIO_DBG_LED90 IMX_GPIO_NR(2, 0)
	MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL), /* DBG LED90 */
#define GPIO_DBG_LED91 IMX_GPIO_NR(2, 1)
	MX6_PAD_NANDF_D1__GPIO2_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL), /* DBG LED91 */
#define GPIO_DBG_LED92 IMX_GPIO_NR(2, 2)
	MX6_PAD_NANDF_D2__GPIO2_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), /* DBG LED92 */
#define GPIO_DBG_LED93 IMX_GPIO_NR(2, 3)
	MX6_PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL), /* DBG LED93 */
#ifdef INTERNAL_RFID
        /* NFC RFID reader control */
#define GPIO_NFC_IRQ IMX_GPIO_NR(4, 6)
	MX6_PAD_KEY_COL0__GPIO4_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), /* NFC_IRQ(RFID) / UART4_TX */
#define GPIO_UX_IRQ IMX_GPIO_NR(4, 7)
	MX6_PAD_KEY_ROW0__GPIO4_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL), /* RFID_RSTn / UART4_RX */
#else
#define GPIO_NFC_IRQ IMX_GPIO_NR(4, 6)
	MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL), /* NFC_IRQ(RFID) / UART4_TX */
#define GPIO_UX_IRQ IMX_GPIO_NR(4, 7)
	MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL), /* RFID_RSTn / UART4_RX */
#endif
#define GPIO_UX_GSTIC_RSTn IMX_GPIO_NR(4, 10)
	MX6_PAD_KEY_COL2__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL), /* UX_GSTIC_RSTn */
#define GPIO_UX_GSTIC_TS IMX_GPIO_NR(4, 11)
	MX6_PAD_KEY_ROW2__GPIO4_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), /* UX_GSTIC_TS */
#define GPIO_UX_5VEN IMX_GPIO_NR(4, 14)
	MX6_PAD_KEY_COL4__GPIO4_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL), /* UX_5V_EN */
#define GPIO_UX_5VOC IMX_GPIO_NR(3, 16)
	MX6_PAD_EIM_D16__GPIO3_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL), /* UX_5V_OC */

        /* Ultrasonic Sensor control */
#define GPIO_USON_PWREN IMX_GPIO_NR(4, 24)
	MX6_PAD_DISP0_DAT3__GPIO4_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL), /* USON_PWREN */
	MX6_PAD_SD4_CLK__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL), /* Ultrasonic Sensor 1 */
	MX6_PAD_SD4_CMD__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL), /* Ultrasonic Sensor 1 */
	MX6_PAD_KEY_COL1__UART5_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL), /* Ultrasonic Sensor 2 */
	MX6_PAD_KEY_ROW1__UART5_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL), /* Ultrasonic Sensor 2 */

        /* Flexcan interface control */
#define GPIO_RS485_CAN_EN          IMX_GPIO_NR(1, 6)
	MX6_PAD_GPIO_6__GPIO1_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), /* SER_PWR_EN + RS485 PWR_EN */
	MX6_PAD_GPIO_7__FLEXCAN1_TX | MUX_PAD_CTRL(NO_PAD_CTRL), /* Flexcan1 TX */
	MX6_PAD_GPIO_8__FLEXCAN1_RX | MUX_PAD_CTRL(NO_PAD_CTRL), /* Flexcan1 RX */
        /* Modem Control*/
#define GPIO_MODEM_PWREN IMX_GPIO_NR(3, 23)
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), /* MOD_PWREN */
#define GPIO_MODEM_RESET IMX_GPIO_NR(3, 24)
	MX6_PAD_EIM_D24__GPIO3_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),  /* MOD_RESETn */
#define GPIO_MODEM_IGT IMX_GPIO_NR(7, 13)
	MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL), /* MOD_IGTn */
#define GPIO_MODEM_PWRIND IMX_GPIO_NR(4, 5)
	MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), /* MOD_PWRIND */
        /* Audio Pad control*/
#define GPIO_SPEAKER_PWREN IMX_GPIO_NR(4, 25)
	MX6_PAD_DISP0_DAT4__GPIO4_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL), /* SPK_PWREN */
	MX6_PAD_GPIO_0__CCM_CLKO1 | MUX_PAD_CTRL(PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm), /* AUD_MCLK */
	MX6_PAD_DISP0_DAT20__AUD4_TXC | MUX_PAD_CTRL(AUD_PAD_CTRL),	/* AUD4_TXC */
	MX6_PAD_DISP0_DAT21__AUD4_TXD | MUX_PAD_CTRL(AUD_PAD_CTRL),	/* AUD4_TXD */
	MX6_PAD_DISP0_DAT22__AUD4_TXFS | MUX_PAD_CTRL(AUD_PAD_CTRL),	/* AUD4_TXFS */
	MX6_PAD_DISP0_DAT23__AUD4_RXD | MUX_PAD_CTRL(AUD_PAD_CTRL),	/* AUD4_RXD */
        /* MIPI Camera */
	MX6_PAD_CSI0_DAT4__GPIO5_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* CAM_TRIGGER */
	MX6_PAD_CSI0_DAT5__GPIO5_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* CAM_STROBE */
	MX6_PAD_CSI0_DAT6__GPIO5_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* CAM_RESET_n */
#define GPIO_CAM1_PWRDN IMX_GPIO_NR(5,25)
	MX6_PAD_CSI0_DAT7__GPIO5_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* CAM_PWDN_n */
        /* LVDS LCD control */
#define GPIO_LCD_PWREN IMX_GPIO_NR(4, 21)
	MX6_PAD_DISP0_DAT0__GPIO4_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL), /* LCD Power Enable */
	MX6_PAD_DISP0_DAT1__GPIO4_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL), /* LCD_REV_SCAN */
#define GPIO_LCD_SEL_6o8 IMX_GPIO_NR(4, 23)
	MX6_PAD_DISP0_DAT2__GPIO4_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), /* LCD_SEL_6OR8 */
	MX6_PAD_DISP0_DAT8__PWM1_OUT | MUX_PAD_CTRL(NO_PAD_CTRL), /* LCD PWM */
        /* LVDS 3.3V enable */
#define GPIO_LVDS3V_PWREN IMX_GPIO_NR(4, 30)
	MX6_PAD_DISP0_DAT9__GPIO4_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL), /* LVDS 3.3 V_PWREN*/
        /* CAM2 DeSerializer */
#define GPIO_CAM2_PWREN IMX_GPIO_NR(5,20)
	MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CAM2_PWREN */
	MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CAM2_LFLT_n */
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CAM2_RST */
	MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), /* CAM2_LOCK */
        MX6_PAD_CSI0_DAT12__IPU1_CSI0_DATA12 | MUX_PAD_CTRL(WEAK_PULLDN), /* CAM2 Serial Data */
        MX6_PAD_CSI0_DAT13__IPU1_CSI0_DATA13 | MUX_PAD_CTRL(WEAK_PULLDN), /* CAM2 Serial Data */
        MX6_PAD_CSI0_DAT14__IPU1_CSI0_DATA14 | MUX_PAD_CTRL(CSI_PAD_CTRL), /* CAM2 Serial Data */
        MX6_PAD_CSI0_DAT15__IPU1_CSI0_DATA15 | MUX_PAD_CTRL(CSI_PAD_CTRL), /* CAM2 Serial Data */
        MX6_PAD_CSI0_DAT16__IPU1_CSI0_DATA16 | MUX_PAD_CTRL(CSI_PAD_CTRL), /* CAM2 Serial Data */
        MX6_PAD_CSI0_DAT17__IPU1_CSI0_DATA17 | MUX_PAD_CTRL(CSI_PAD_CTRL), /* CAM2 Serial Data */
        MX6_PAD_CSI0_DAT18__IPU1_CSI0_DATA18 | MUX_PAD_CTRL(CSI_PAD_CTRL), /* CAM2 Serial Data */
        MX6_PAD_CSI0_DAT19__IPU1_CSI0_DATA19 | MUX_PAD_CTRL(CSI_PAD_CTRL), /* CAM2 Serial Data */
        MX6_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK | MUX_PAD_CTRL(CSI_PAD_CTRL), /* CAM2 PIXCLK */
	MX6_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC | MUX_PAD_CTRL(NO_PAD_CTRL),	/* CAM2 Hsync */
	MX6_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC | MUX_PAD_CTRL(NO_PAD_CTRL),	/* CAM2 VSync */
};

static void setup_iomux_enet(void)
{
    imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset LAN PHY */
	gpio_direction_output(GPIO_ETH_PHY_RESET , 0);
	udelay(500); /* VG : Need to tune eth startup delay */ 
	gpio_set_value(GPIO_ETH_PHY_RESET, 1);
}

static void enable_lcd(void)
{
	gpio_direction_output(GPIO_LCD_PWREN, 1);
}

static void disable_lcd(void)
{
	gpio_direction_output(GPIO_LCD_PWREN, 0);
}

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

/* I2C2, HDMI  */
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

/* I2C3, Usonic, CSI Camera, I2S Audio */
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

iomux_v3_cfg_t const hdmi_pads[] = {
	MX6_PAD_EIM_A25__HDMI_TX_CEC_LINE,	/* HDMI_TX_CEC_LINE */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_rs485_pads, ARRAY_SIZE(uart2_rs485_pads));
}

#define CONFIG_FSL_ESDHC
#ifdef CONFIG_FSL_ESDHC
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
#ifdef CONFIG_TARGET_MX6QPSABRESD_CPNK
        return CONFIG_SYS_MMC_ENV_DEV;
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

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

    switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC3 is always present on cpnk board*/
		break;
	case USDHC1_BASE_ADDR:
		ret = 1; /* WLAN chip always present on cpnk board */
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
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
                        usdhc_cfg[0].max_bus_width = 8; /* eMMC bus width */
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
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
#endif

int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

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

#if defined(CONFIG_VIDEO_IPUV3)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

        disable_lcd(); /* Disable LCD */
	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);

}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static int detect_i2c(struct display_info_t const *dev)
{
        return ((0 == i2c_set_bus_num(dev->bus))
                &&
                (0 == i2c_probe(dev->addr)));
}


static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
        enable_lcd(); /* Enable LCD */
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

struct display_info_t const displays[] = {
{
        .bus    = 0,
        .addr   = 0,
        .pixfmt = IPU_PIX_FMT_RGB24,
        .detect = NULL,
        .enable = enable_lvds,
        .mode   = {
                .name           = "AM-1024768Y2TZQW-A2H",
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .pixclock       = 15385,
                .left_margin    = 220,
                .right_margin   = 40,
                .upper_margin   = 21,
                .lower_margin   = 7,
                .hsync_len      = 60,
                .vsync_len      = 10,
                .sync           = FB_SYNC_EXT,
                .vmode          = FB_VMODE_NONINTERLACED
        }
},
{
        .bus    = 0,
        .addr   = 0,
        .pixfmt = IPU_PIX_FMT_RGB24,
        .detect = NULL,
        .enable = enable_lvds,
        .mode   = {
                .name           = "KYOCERA-TCG104XGLPAPNN-AN40",
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .pixclock       = 15385,
                .left_margin    = 220,
                .right_margin   = 40,
                .upper_margin   = 21,
                .lower_margin   = 7,
                .hsync_len      = 60,
                .vsync_len      = 10,
                .sync           = FB_SYNC_EXT,
                .vmode          = FB_VMODE_NONINTERLACED
        }
},
{
        .bus    = 1,
        .addr   = 0x50,
        .pixfmt = IPU_PIX_FMT_RGB24,
        .detect = detect_i2c,
        .enable = do_enable_hdmi,
        .mode   = {
                .name           = "HDMI",
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .pixclock       = 15385,
                .left_margin    = 220,
                .right_margin   = 40,
                .upper_margin   = 21,
                .lower_margin   = 7,
                .hsync_len      = 60,
                .vsync_len      = 10,
                .sync           = FB_SYNC_EXT,
                .vmode          = FB_VMODE_NONINTERLACED
        }
}
};

size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

        /* CPNK HDMI TX_CEC_LINE */
	imx_iomux_v3_setup_multiple_pads(hdmi_pads, ARRAY_SIZE(hdmi_pads));

	enable_ipu_clock();
	imx_setup_hdmi();
	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

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
	setup_iomux_enet();
	return cpu_eth_init(bis);
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
#define GPIO_OTG_PWREN  IMX_GPIO_NR(3, 22)
	MX6_PAD_EIM_D22__USB_OTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
	MX6_PAD_EIM_D21__USB_OTG_OC | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usb_hc1_pads[] = {
#define GPIO_USB_PWREN  IMX_GPIO_NR(1, 4)
	MX6_PAD_GPIO_4__GPIO1_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define GPIO_USB_RESET  IMX_GPIO_NR(1, 5)
	MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_usb(void)
{
    imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));

	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 *
         * OTG ID daisy chain set
         * 0: ENET_RX_ER as USB_OTG_ID
         * 1: GPIO_1 as USB_OTG_ID
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 1);

	imx_iomux_v3_setup_multiple_pads(usb_hc1_pads,
					 ARRAY_SIZE(usb_hc1_pads));
}

int board_ehci_hcd_init(int port)
{
        u32 *usbnc_usb_ctrl;

        if (port > 1)
                return -EINVAL;

        usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
                                 port * 4);

        setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

        /* Reset USB hub */ 
        gpio_direction_output(GPIO_USB_RESET, 0);
        mdelay(2);
        gpio_direction_output(GPIO_USB_RESET, 1);
	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(IMX_GPIO_NR(1, 4), 1);
		else
			gpio_direction_output(IMX_GPIO_NR(1, 4), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif
static unsigned gpios_output_low[] = {
        GPIO_WIF_PWREN,
        GPIO_WIF_RESET,
        GPIO_USON_PWREN,
        GPIO_UX_5VEN, /* UX Power low on bootup */
        GPIO_UX_GSTIC_RSTn,
        GPIO_UX_GSTIC_TS,
        GPIO_MODEM_PWREN,
        GPIO_MODEM_IGT,     /* MOD_IGTn. Modem power off requires IGNn to low and modem reset to high */
        GPIO_RS485_TX_EN,
        GPIO_RS485_RX_EN,
        GPIO_DBG_LED91,
        GPIO_DBG_LED92,
        GPIO_DBG_LED93,
        GPIO_CAM2_PWREN,
        GPIO_CAM1_PWRDN,
};

static const char *gpios_output_low_names[] = {
        "GPIO_WIF_PWREN",
        "GPIO_WIF_RESET",
        "GPIO_USON_PWREN",
        "GPIO_UX_5VEN",
        "GPIO_UX_GSTIC_RSTn",
        "GPIO_UX_GSTIC_TS",
        "GPIO_MODEM_PWREN",
        "GPIO_MODEM_IGT",
        "GPIO_RS485_TX_EN",
        "GPIO_RS485_RX_EN",
        "GPIO_DBG_LED91",
        "GPIO_DBG_LED92",
        "GPIO_DBG_LED93",
        "GPIO_CAM2_PWREN",
        "GPIO_CAM1_PWRDN",
};

static unsigned gpios_output_high[] = {
        GPIO_PMIC_EN,
        GPIO_LG_CHGEN,
        GPIO_LVDS3V_PWREN,
        GPIO_MODEM_RESET,     /* MOD_RESETn. Modem power off requires IGNn to low and modem reset to high */
        GPIO_USB_RESET,
        GPIO_USB_PWREN,
        GPIO_OTG_PWREN,
        GPIO_RS485_CAN_EN,
        GPIO_DBG_LED90,
        GPIO_SPEAKER_PWREN,
        GPIO_LCD_SEL_6o8,
        GPIO_LCD_PWREN,
};

static const char *gpios_output_high_names[] = {
        "GPIO_PMIC_EN",
        "GPIO_LG_CHGEN",
        "GPIO_LVDS3V_PWREN",
        "GPIO_MODEM_RESET",
        "GPIO_USB_RESET",
        "GPIO_USB_PWREN",
        "GPIO_OTG_PWREN",
        "GPIO_RS485_CAN_EN",
        "GPIO_DBG_LED90",
        "GPIO_SPEAKER_PWREN",
        "GPIO_LCD_SEL_6o8",
        "GPIO_LCD_PWREN",
};

static const unsigned gpios_input[] = {
        GPIO_LG_CHGPFW,
        GPIO_MODEM_PWRIND,
        GPIO_LG_CAPGOOD,
#ifdef INTERNAL_RFID
        GPIO_NFC_IRQ,
        GPIO_UX_IRQ,
#endif
        GPIO_UX_5VOC,
};

static const char *gpios_input_names[] = {
        "GPIO_LG_CHGPFW",
        "GPIO_MODEM_PWRIND",
        "GPIO_LG_CAPGOOD",
#ifdef INTERNAL_RFID
        "GPIO_NFC_IRQ",
        "GPIO_UX_IRQ",
#endif
        "GPIO_UX_5VOC",
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
	setup_iomux_uart();
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

    /* Setup CPNK misc init pads */
    imx_iomux_v3_setup_multiple_pads(cpnk_init_pads, ARRAY_SIZE(cpnk_init_pads));
    /* Device specific GPIO settings */

    gpios_request(gpios_input, gpios_input_names, ARRAY_SIZE(gpios_input));
    gpios_request(gpios_output_high, gpios_output_high_names, ARRAY_SIZE(gpios_output_high));
    gpios_request(gpios_output_low, gpios_output_low_names, ARRAY_SIZE(gpios_output_low));

    set_gpios_input(gpios_input, ARRAY_SIZE(gpios_input));
    set_gpios(gpios_output_high, ARRAY_SIZE(gpios_output_high), 1);
    set_gpios(gpios_output_low, ARRAY_SIZE(gpios_output_low), 0);
#if defined(CONFIG_VIDEO_IPUV3)
    setup_display();
#endif

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
        /* Enabling access to PMIC on I2C */
	gpio_direction_output(GPIO_PMIC_EN, 1);

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif
	return 0;
}

static struct pmic *pfuze;
int power_init_board(void)
{
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

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	gpio_direction_output(GPIO_DBG_LED91, 1);

        env = env_get("boardtype");
        if(!env){
               set_default_env(NULL);
               env_set("boardtype","CPNK");
               printf("Saving env in flash \n");
               env_save();
        }
        printf("Reset Cause 0x%08X \n",get_imx_reset_cause());
        env_set_ulong("reset_cause", get_imx_reset_cause());
	return 0;
}

int checkboard(void)
{
	puts("Board: ChargePoint CPNK\n");
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
