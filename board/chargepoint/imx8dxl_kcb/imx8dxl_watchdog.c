/*
 * imx8dxl_watchdog.c - driver for i.mx SCFW timer-based watchdog
 *
 * Based on the imx_watchdog.c and the u-boot driver model docs.
 *
 */

//TODO RMW - what should header say re Copyright when based on NXP's prior work?

#include <common.h>
#include <dm.h>
#include <wdt.h>
#include <watchdog.h>
#include <misc.h>
#include <linux/arm-smccc.h>
#include <asm/arch/sci/sci.h>
#include <time.h>

#define IMX_SIP_TIMER			0xC2000002
#define IMX_SIP_TIMER_START_WDOG		0x01
#define IMX_SIP_TIMER_STOP_WDOG		0x02
#define IMX_SIP_TIMER_SET_WDOG_ACT	0x03
#define IMX_SIP_TIMER_PING_WDOG		0x04
#define IMX_SIP_TIMER_SET_TIMEOUT_WDOG	0x05
#define IMX_SIP_TIMER_GET_WDOG_STAT	0x06
#define IMX_SIP_TIMER_SET_PRETIME_WDOG	0x07

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_WDT)
static int imx_sc_wdt_ping(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(IMX_SIP_TIMER, IMX_SIP_TIMER_PING_WDOG,
		      0, 0, 0, 0, 0, 0, &res);

	return 0;
}

static int imx_sc_wdt_set_timeout(uint32_t timeout_ms)
{
	struct arm_smccc_res res;

	arm_smccc_smc(IMX_SIP_TIMER, IMX_SIP_TIMER_SET_TIMEOUT_WDOG,
		      timeout_ms, 0, 0, 0, 0, 0, &res);

	return res.a0 ? -EACCES : 0;
}

static int imx_sc_wdt_start(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(IMX_SIP_TIMER, IMX_SIP_TIMER_START_WDOG,
		      0, 0, 0, 0, 0, 0, &res);
    if (res.a0) {
            printk(KERN_ALERT "  uboot imx_sc_wdt_start() Failed START_WDOG with %lu\n", res.a0);
            return -EACCES;
    }

	arm_smccc_smc(IMX_SIP_TIMER, IMX_SIP_TIMER_SET_WDOG_ACT,
		      SC_TIMER_WDOG_ACTION_PARTITION,
		      0, 0, 0, 0, 0, &res);
    if (res.a0) {
            printk(KERN_ALERT "  uboot imx_sc_wdt_start() Failed SET_WDOG_ACT with %lu\n", res.a0);
            return -EACCES;
    }
    else
            return 0;
}

static int imx_sc_wdt_get_status(uint32_t* timeoutValue, uint32_t* remainingTime)
{
	struct arm_smccc_res res;

	arm_smccc_smc(IMX_SIP_TIMER, IMX_SIP_TIMER_GET_WDOG_STAT,
		      0, 0, 0, 0, 0, 0, &res);

	if (!res.a0) {
		if (timeoutValue) {
			*timeoutValue = (uint32_t)res.a1;
		}
		if (remainingTime) {
			*remainingTime = (uint32_t)res.a3;
		}
	}

	return res.a0 ? -EACCES : 0;
}

struct imx8dxl_wdt_priv {
	struct udevice *baseDevice;
	bool disableResetPings;
	uint32_t timeout;
};

static int imx8dxl_wdt_reset(struct udevice *dev)
{
#ifndef CONFIG_WATCHDOG_RESET_DISABLE
	struct imx8dxl_wdt_priv *priv = dev_get_priv(dev);

	if (!priv->disableResetPings) {
		imx_sc_wdt_ping();
	}
#endif /* CONFIG_WATCHDOG_RESET_DISABLE*/

	return 0;
}

// timeout==0 implies taking priv->timeout instead.
static void imx8dxl_watchdog_init(struct udevice *dev, u64 timeout)
{
	struct imx8dxl_wdt_priv *priv = dev_get_priv(dev);

	if (timeout) {
		imx_sc_wdt_set_timeout((uint32_t)timeout);
	}
	else {
		imx_sc_wdt_set_timeout(priv->timeout);
	}

	imx_sc_wdt_start();

	// Reset-Ping once
	imx8dxl_wdt_reset(dev);
}

static int imx8dxl_wdt_expire_now(struct udevice *dev, ulong flags)
{
	struct imx8dxl_wdt_priv *priv = dev_get_priv(dev);

	printf("imx8dxl_wdt_expire_now() - Watchdog timeout=%u. Will need to wait that long...\n", priv->timeout);

	// Stop the 'reset/ping' from executing any more.
	priv->disableResetPings = true;

	// Because expire_now isn't supported for SCFW:
	// Shut off the reset via priv->disableResetPings (no more pings given to FW)
	// Print that expiration will be delayed by "remaining"
	// Print an update once per 'period' decrement of 'remaining' as a countdown to zero.

	const unsigned long period = 2 * 1000 * 1000;	// Two seconds in uSeconds
	unsigned long tLastUSec = timer_get_us();
	uint32_t remainingTime=0;

	while (1) {
		if (timer_get_us() > tLastUSec + period) {
			tLastUSec = timer_get_us();
			imx_sc_wdt_get_status(NULL, &remainingTime);
			printf("  Watchdog expiration/reboot in: %3u seconds.\n", remainingTime / (uint32_t)1000);
		}
	}

	return 0;
}

static int imx8dxl_wdt_start(struct udevice *dev, u64 timeout, ulong flags)
{
	imx8dxl_watchdog_init(dev, 0);

	return 0;
}

static int imx8dxl_wdt_probe(struct udevice *dev)
{
	struct imx8dxl_wdt_priv *priv = dev_get_priv(dev);

	priv->baseDevice = dev;
	if (!priv->baseDevice)
		return -ENOENT;

	priv->disableResetPings = false;

#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 128000
#endif
	priv->timeout = CONFIG_WATCHDOG_TIMEOUT_MSECS;
	assert(priv->timeout);

	return 0;
}

static const struct wdt_ops imx8dxl_wdt_ops = {
	.start		= imx8dxl_wdt_start,
	.reset		= imx8dxl_wdt_reset,
	.expire_now	= imx8dxl_wdt_expire_now,
};

static const struct udevice_id imx8dxl_wdt_ids[] = {
//TODO RMW
	{ .compatible = "fsl,imx8-wdt" },		// Right way? Needs dtsi notations to match?
//	{ .compatible = "fsl,imx-sc-wdt" },		// Right way? Needs dtsi notations to match?
	{}
};

U_BOOT_DRIVER(imx8dxl_wdt) = {
	.name		= "imx8dxl_wdt",
	.id		= UCLASS_WDT,
	.of_match	= imx8dxl_wdt_ids,
	.probe		= imx8dxl_wdt_probe,
	.ops		= &imx8dxl_wdt_ops,
	.priv_auto = sizeof(struct imx8dxl_wdt_priv),
	.flags		= DM_FLAG_PRE_RELOC,
};
#endif
