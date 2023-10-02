# (C) Copyright 2015 Freescale Semiconductor, Inc.
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y  := imx6cpnk.o pin_mux.o
obj-y  += ../../freescale/common/

CFLAGS_pin_mux.o += -I$(srctree)/board/$(VENDOR)/common

extra-$(CONFIG_USE_IMXIMG_PLUGIN) := plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@

extra-y += ../../../arch/$(ARCH)/dts/imx6qp-chargepoint-cpnk.dts
$(obj)/../../../arch/$(ARCH)/dts/imx6qp-chargepoint-cpnk.dts: \
		$(src)/imx6qp-chargepoint-cpnk.dts
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/prod-pubkey-cpnk.dtsi
$(obj)/../../../arch/$(ARCH)/dts/prod-pubkey-cpnk.dtsi: \
		$(src)/prod-pubkey-cpnk.dtsi
	ln -fsn $(abspath $<) $@
