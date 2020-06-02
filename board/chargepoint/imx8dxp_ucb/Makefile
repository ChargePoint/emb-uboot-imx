#
# Copyright 2017 NXP
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += imx8dxp_ucb.o
obj-y += ../../freescale/common/

extra-y += ../../../arch/$(ARCH)/dts/imx8dxp-chargepoint-ucb.dts
$(obj)/../../../arch/$(ARCH)/dts/imx8dxp-chargepoint-ucb.dts: \
		$(src)/imx8dxp-chargepoint-ucb.dts
	ln -fsn $(abspath $<) $@

