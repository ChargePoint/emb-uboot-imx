/*
 * Copyright 2022 ChargePoint, Inc.
 *
 * General helper functions to convert keys in the fdt with a
 * prefix to keys that will be checked by the fitimage bootcode.
 *
 * Because this is passed as an environment variable, there can be
 * no spaces or special characters in the reason.
 *
 * This particular feature is for ChargePoint.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _CHARGEPOINT_BOOTREASON_H_
#define _CHARGEPOINT_BOOTREASON_H_

#ifdef CONFIG_IMX8

static inline const char *get_wdog_reset_reason(void)
{
	sc_err_t ret;
	sc_pm_reset_reason_t reason;

	ret = sc_pm_reset_reason(SC_IPC_CH, &reason);
	if (ret == SC_ERR_NONE) {
		switch (reason) {
		case SC_PM_RESET_REASON_POR:
			return "POR";
		case SC_PM_RESET_REASON_JTAG:
			return "JTAG";
		case SC_PM_RESET_REASON_SW:
			return "SW";
		case SC_PM_RESET_REASON_WDOG:
			return "WDOG";
		case SC_PM_RESET_REASON_LOCKUP:
			return "LOCKUP";
		case SC_PM_RESET_REASON_SNVS:
			return "SNVS";
		case SC_PM_RESET_REASON_TEMP:
			return "TEMP";
		case SC_PM_RESET_REASON_MSI:
			return "MSI";
		case SC_PM_RESET_REASON_UECC:
			return "UECC";
		case SC_PM_RESET_REASON_SCFW_WDOG:
			return "SCFW_WDOG";
		case SC_PM_RESET_REASON_ROM_WDOG:
			return "ROM_WDOG";
		case SC_PM_RESET_REASON_SECO:
			return "SECO";
		case SC_PM_RESET_REASON_SCFW_FAULT:
			return "SCFW_FAULT";
		default:
			break;
		}
	}

	return "Unknown";
}

#else /* MX5 MX6 MX7 MX8M */

static inline const char *get_wdog_reset_reason(void)
{
	u32 cause;
#ifdef CONFIG_MX6
	unsigned int wrsr;
#endif
#if defined(CONFIG_DISPLAY_CPUINFO) && !defined(CONFIG_SPL_BUILD)
	cause = get_imx_reset_cause();
#else
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;

	/* Reset register is not always cleared, so read and reset */
	cause = readl(&src_regs->srsr);
	writel(cause, &src_regs->srsr);
#endif
	switch (cause) {
	case 0x00001:
	case 0x00011:
		return "POR";
	case 0x00004:
		return "CSU";
	case 0x00008:
		return "IPP_USER";
	case 0x00010:
#ifdef  CONFIG_MX7
		return "WDOG1";
#else
#ifdef CONFIG_MX6
		wrsr = readw(&((struct wdog_regs *)WDOG1_BASE_ADDR)->wrsr);
		if (wrsr & (1 << 0)) {
			return "SW";
		} else if (wrsr & (1 << 4)) {
			return "POR";
		}
#endif
		return "WDOG";
#endif
	case 0x00020:
		return "JTAG_HIGH-Z";
	case 0x00040:
		return "JTAG_SW";
	case 0x00080:
		return "WDOG3";
#ifdef CONFIG_MX7
	case 0x00100:
		return "WDOG4";
	case 0x00200:
		return "TEMPSENSE";
#elif defined(CONFIG_IMX8M)
	case 0x00100:
		return "WDOG2";
	case 0x00200:
		return "TEMPSENSE";
#else
	case 0x00100:
		return "TEMPSENSE";
	case 0x10000:
		return "WARM_BOOT";
#endif
	default:
		break;
	}

	return "Unknown";
}

#endif /* CONFIG_IMX8 */

#endif /* _CHARGEPOINT_BOOTREASON_H_ */

