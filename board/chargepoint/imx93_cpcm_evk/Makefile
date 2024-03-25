#
# Copyright 2022 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx93_cpcm_evk.o
obj-y += ../../freescale/common/

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_IMX9_DRAM_INLINE_ECC
obj-$(CONFIG_CHARGEPOINT_IMX93_CPCM_EVK_LPDDR4X) += lpddr4x_timing_ecc.o lpddr4x_timing_1866mts_ecc.o
else
obj-$(CONFIG_CHARGEPOINT_IMX93_CPCM_EVK_LPDDR4X) += lpddr4x_timing.o lpddr4x_timing_1866mts.o
endif
else
extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@
endif
