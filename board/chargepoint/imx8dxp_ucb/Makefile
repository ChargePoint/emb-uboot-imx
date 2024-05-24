#
# Copyright 2020 Chargepoint
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += imx8dxp_ucb.o
obj-$(CONFIG_WDT) += ../common/imx8x_watchdog.o
obj-y += ../../freescale/common/

extra-y += ../../../arch/$(ARCH)/dts/imx8dxp-chargepoint-ucb.dts
$(obj)/../../../arch/$(ARCH)/dts/imx8dxp-chargepoint-ucb.dts: \
		$(src)/imx8dxp-chargepoint-ucb.dts
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/ucb-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/ucb-pubkey.dtsi: \
		$(src)/ucb-pubkey.dtsi
	ln -fsn $(abspath $<) $@
