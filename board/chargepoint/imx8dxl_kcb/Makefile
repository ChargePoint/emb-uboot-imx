#
# Copyright 2020 NXP
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += imx8dxl_kcb.o
obj-y += ../../freescale/common/

ifdef CONFIG_SPL_BUILD
obj-$(CONFIG_SPL_BUILD) += spl.o
else
extra-y += ../../../arch/$(ARCH)/dts/imx8dxl-chargepoint-kcb.dts
$(obj)/../../../arch/$(ARCH)/dts/imx8dxl-chargepoint-kcb.dts: \
		$(src)/imx8dxl-chargepoint-kcb.dts
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/prod-pubkey-kcb.dtsi
$(obj)/../../../arch/$(ARCH)/dts/prod-pubkey-kcb.dtsi: \
		$(src)/prod-pubkey-kcb.dtsi
	ln -fsn $(abspath $<) $@

endif
