#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8mm_mcb.o
obj-y += ../../freescale/common/

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
else

extra-y += ../../../arch/$(ARCH)/dts/imx8mm-chargepoint-mcb.dts
$(obj)/../../../arch/$(ARCH)/dts/imx8mm-chargepoint-mcb.dts: \
		$(src)/imx8mm-chargepoint-mcb.dts
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/prod-pubkey-mcb.dtsi
$(obj)/../../../arch/$(ARCH)/dts/prod-pubkey-mcb.dtsi: \
		$(src)/prod-pubkey-mcb.dtsi
	ln -fsn $(abspath $<) $@

endif
