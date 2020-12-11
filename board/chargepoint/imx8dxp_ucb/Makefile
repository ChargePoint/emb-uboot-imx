#
# Copyright 2020 Chargepoint
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += imx8dxp_ucb.o
obj-y += ../../freescale/common/

extra-y += ../../../arch/$(ARCH)/dts/imx8dxp-chargepoint-ucb.dts
$(obj)/../../../arch/$(ARCH)/dts/imx8dxp-chargepoint-ucb.dts: \
		$(src)/imx8dxp-chargepoint-ucb.dts
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/prod-pubkey-ucb.dtsi
$(obj)/../../../arch/$(ARCH)/dts/prod-pubkey-ucb.dtsi: \
		$(src)/prod-pubkey-ucb.dtsi
	ln -fsn $(abspath $<) $@
