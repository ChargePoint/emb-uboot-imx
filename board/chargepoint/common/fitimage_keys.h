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

#include <fuse.h>

#if defined(CONFIG_ARCH_IMX8M)
struct imx_sec_config_fuse_t {
	int bank;
	int word;
};

struct imx_sec_config_fuse_t const imx_sec_config_fuse = {
	.bank = 1,
	.word = 3,
};

#define HAB_ENABLED_BIT (is_soc_type(MXC_SOC_IMX8M)? 0x2000000 : 0x2)

/* Check hab status, this is basically copied from imx_hab_is_enabled() */
bool hab_is_enabled(void)
{

	struct imx_sec_config_fuse_t *fuse =
		(struct imx_sec_config_fuse_t *)&imx_sec_config_fuse;
	uint32_t reg;
	int ret;

	ret = fuse_read(fuse->bank, fuse->word, &reg);
	if (ret) {
		printf("\nSecure boot fuse read error!\n");
		return false;
	}

	if (!((reg & HAB_ENABLED_BIT) == HAB_ENABLED_BIT)) {
		return false;
	} else {
		return true;
	}
}
#endif

static inline void setup_fitimage_keys(void)
{
#if CONFIG_IS_ENABLED(FIT_SIGNATURE)
	int noffset;
	int sig_node;
	void *sig_blob;
	const char *sig_prefix;

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
		//sc_ipc_t ipcHndl = gd->arch.ipc_channel_handle;

		err = sc_seco_chip_info(-1, &lc, NULL, NULL, NULL);
		if ((err == SC_ERR_NONE) && (lc == 0x80)) {
			sig_prefix = "prod:";
		} else {
			sig_prefix = "dev:";
		}
	} while(0);
#else
	sig_prefix = hab_is_enabled() ? "prod:" : "dev:";
#endif
	sig_blob = (void *)(uintptr_t)gd->fdt_blob;

	/* process the signature nodes */
	sig_node = fdt_subnode_offset(sig_blob, 0, FIT_SIG_NODENAME);

	if (sig_node < 0) {
		printf("%s: No signature node found: %s\n",
		      __func__, fdt_strerror(sig_node));
		return;
	}
	fdt_for_each_subnode(noffset, sig_blob, sig_node) {
		const char *required;
		char buf[16];
		int ret;

		required = fdt_getprop(sig_blob, noffset, "required", NULL);
		if ((required == NULL) ||
		    (strncmp(required, sig_prefix, strlen(sig_prefix)) != 0))
			continue;

		/*
		 * Copy string suffix to stack buffer before calling
		 * ftd_setprop_string().  Otherwise the string may be truncated while
		 * being set.
		 */
		memset(buf, 0, sizeof(buf));
		strncpy(buf, &required[strlen(sig_prefix)], sizeof(buf));
		ret = fdt_setprop_string(sig_blob, noffset, "required", buf);
		if (ret) {
			printf("Failed to update required signature '%s'\n",
			       fit_get_name(sig_blob, noffset, NULL));
		}
	}
#endif

	return;
}

#endif /* _CHARGEPOINT_FITIMAGE_KEYS_H_ */

