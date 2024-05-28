/*
 * Copyright 2020 ChargePoint, Inc.
 *
 * General helper functions to convert keys in the fdt with a
 * prefix to keys that will be checked by the fitimage bootcode.
 *
 * This particular feature is for ChargePoint.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _CHARGEPOINT_FITIMAGE_KEYS_H_
#define _CHARGEPOINT_FITIMAGE_KEYS_H_

#if CONFIG_IS_ENABLED(CHARGEPOINT_MFG)
#define SIGNATURE_PREFIX_QA "mfg-qa:"
#define SIGNATURE_PREFIX_PROD "mfg-prod:"
#else
#define SIGNATURE_PREFIX_QA "qa:"
#define SIGNATURE_PREFIX_PROD "prod:"
#endif

#include <image.h>
#include <asm/mach-imx/hab.h>

static inline void setup_fitimage_keys(void)
{
#if CONFIG_IS_ENABLED(FIT_SIGNATURE)
	int noffset;
	int sig_node;
	void *sig_blob;
	int allowany;
	const char sig_prefix_prod[] = SIGNATURE_PREFIX_PROD;
	const char sig_prefix_qa[] = SIGNATURE_PREFIX_QA;
	const char sig_prefix_dev[] = "dev:";

	/*
	 * This will change the "u-boot" fdt required keys that
	 * have been compiled in.
	 *
	 * The code must match the key checks in the common image-fit
	 * verification using the string match. By stripping, the
	 * prefix of a key it then turns that into a required match
	 * for the fit verification
	 */
#if defined(CONFIG_ARCH_IMX8)
	do {
		sc_err_t err;
		uint16_t lc;
		sc_ipc_t ipcHndl = gd->arch.ipc_channel_handle;

		err = sc_seco_chip_info(ipcHndl, &lc, NULL, NULL, NULL);
		if ((err == SC_ERR_NONE) && (lc == 0x80)) {
			allowany = 0;
		} else {
			allowany = 1;
		}
	} while(0);
#else
	allowany = imx_hab_is_enabled() ? 0 : 1;
#endif
	sig_blob = (void *)(uintptr_t)gd->fdt_blob;

	/* process the signature nodes */
	sig_node = fdt_subnode_offset(sig_blob, 0, FIT_SIG_NODENAME);
	if (sig_node < 0) {
		debug("%s: No signature node found: %s\n",
		      __func__, fdt_strerror(sig_node));
		return;
	}
	fdt_for_each_subnode(noffset, sig_blob, sig_node) {
		const char *required;
		char buf[16];
		int ret;

		required = fdt_getprop(sig_blob, noffset, "required", NULL);
		if (required == NULL) {
			continue;
		}
		if (strncmp(required,
			    sig_prefix_prod, strlen(sig_prefix_prod)) == 0) {
			/*
			 * Copy string suffix to stack buffer before calling
			 * ftd_setprop_string().  Otherwise the string may be
			 * truncated while being set.
			 */
			memset(buf, 0, sizeof(buf));
			strncpy(buf, &required[strlen(sig_prefix_prod)],
				sizeof(buf));
			ret = fdt_setprop_string(sig_blob, noffset,
						 "required", buf);
			if (ret) {
				printf("Failed to update signature '%s'\n",
				       fit_get_name(sig_blob, noffset, NULL));
			}
		}
		if ((allowany == 1) &&
		    (strncmp(required,
			     sig_prefix_qa, strlen(sig_prefix_qa)) == 0)) {
			/* same copy reason as above */
			memset(buf, 0, sizeof(buf));
			strncpy(buf, &required[strlen(sig_prefix_qa)],
				sizeof(buf));
			ret = fdt_setprop_string(sig_blob, noffset,
						 "required", buf);
			if (ret) {
				printf("Failed to update signature '%s'\n",
				       fit_get_name(sig_blob, noffset, NULL));
			}
		}
		if ((allowany == 1) &&
		    (strncmp(required,
			     sig_prefix_dev, strlen(sig_prefix_dev)) == 0)) {
			/* same copy reason as above */
			memset(buf, 0, sizeof(buf));
			strncpy(buf, &required[strlen(sig_prefix_dev)],
				sizeof(buf));
			ret = fdt_setprop_string(sig_blob, noffset,
						 "required", buf);
			if (ret) {
				printf("Failed to update signature '%s'\n",
				       fit_get_name(sig_blob, noffset, NULL));
			}
		}
	}
#endif

	return;
}

#endif /* _CHARGEPOINT_FITIMAGE_KEYS_H_ */
