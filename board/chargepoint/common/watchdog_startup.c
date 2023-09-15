/* Chargepoint Watchdog u-boot startup - Bob Wolff 2023 */
/*
 * Copyright 2023 ChargePoint, Inc.
 *
 */

#include <common.h>
#include <command.h>
#include <dm.h>
//#include <watchdog.h>
#include <wdt.h>

static struct udevice* currWDTDev = NULL;

#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS   (128 * 1000)
#endif

/**
 * @brief chptStartWatchdog does a step-by-step investigation to obtain the watchdog
 *        driver (if one is present) and then utilizes it to ensure watchdog is 'on'.
 * 
 * */
void chptStartWatchdog(void)
{
	int ret;
	struct udevice *dev;
	struct uclass *uc;

	if (currWDTDev) {
		printf("chptStartWatchdog: Watchdog handle currWDTDev already initialized.\n");
		return;
	}
	
	ret = uclass_get(UCLASS_WDT, &uc);
	if (ret) {
		printf("chptStartWatchdog: Unable to find Watchdog Driver Class UCLASS_WDT. Err=%d\n", ret);
		return;
	}
	else {
		printf("Watchdog class drivers list:\n");
		uclass_foreach_dev(dev, uc) {
			printf("    %s (%s)\n", dev->name, dev->driver->name);
			// Use the first entry we find.
			if (!currWDTDev)
				currWDTDev = dev;
		}

		if (currWDTDev) {
			printf("Utilizing the first watchdog entry - name: %s\n", currWDTDev->name);
		}
		else {
			printf("chptStartWatchdog: Failed to find active watchdog driver.\n");
			return;
		}
	}

	// Now 'start' the timer.
	assert(currWDTDev);
	printf("chptStartWatchdog: Starting watchdog with timeout of: %u\n", (uint32_t)CONFIG_WATCHDOG_TIMEOUT_MSECS);
	ret = wdt_start(currWDTDev, (u64)CONFIG_WATCHDOG_TIMEOUT_MSECS, (ulong)0);
	if (ret == -ENOSYS) {
		printf("chptStartWatchdog: Starting watchdog timer is not supported.\n");
		return;
	} else if (ret) {
		printf("chptStartWatchdog: Starting watchdog timer failed (%d)\n", ret);
		return;
	}

	printf("Watchdog started with %dms timeout.\n", CONFIG_WATCHDOG_TIMEOUT_MSECS);
}

/**
 * @brief chptResetWatchdog utilizes the previously obtained currWDTDev (if non-null)
 *        to feed the watchdog by way of 'wdt_reset' with that handle. If the handle
 *        is null, the function returns.
 * 
 * */
void chptResetWatchdog(void)
{
	int ret;

	if (!currWDTDev) {
		return;
	}
	else {
		ret = wdt_reset(currWDTDev);
		if (ret) {
			printf("chptResetWatchdog: Error while resetting watchdog timer.\n");
			return;
		}
	}
}
