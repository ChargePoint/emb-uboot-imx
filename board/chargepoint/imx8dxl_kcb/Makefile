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
endif
