#
# Copyright 2022 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx93_cpcm.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX93_CPCM_LPDDR4X) += lpddr4x_timing.o
endif

extra-y += ../../../arch/$(ARCH)/dts/imx93-chargepoint-cpcm.dts
$(obj)/../../../arch/$(ARCH)/dts/imx93-chargepoint-cpcm.dts: \
		$(src)/imx93-chargepoint-cpcm.dts
	ln -fsn $(abspath $<) $@
