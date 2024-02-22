#
# Copyright 2020 NXP
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += imx8dxl_pmc.o
obj-y += ../common/imx8x_watchdog.o
obj-y += ../../freescale/common/

extra-y += ../../../arch/$(ARCH)/dts/imx8dxl-chargepoint-pmc.dts
$(obj)/../../../arch/$(ARCH)/dts/imx8dxl-chargepoint-pmc.dts: \
		$(src)/imx8dxl-chargepoint-pmc.dts
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/prod-pubkey-pmc.dtsi
$(obj)/../../../arch/$(ARCH)/dts/prod-pubkey-pmc.dtsi: \
		$(src)/prod-pubkey-pmc.dtsi
	ln -fsn $(abspath $<) $@
