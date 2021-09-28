#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8mm_sbc.o
obj-y += ../../freescale/common/

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
else

extra-y += ../../../arch/$(ARCH)/dts/imx8mm-chargepoint-sbc.dts
$(obj)/../../../arch/$(ARCH)/dts/imx8mm-chargepoint-sbc.dts: \
		$(src)/imx8mm-chargepoint-sbc.dts
	ln -fsn $(abspath $<) $@

endif
