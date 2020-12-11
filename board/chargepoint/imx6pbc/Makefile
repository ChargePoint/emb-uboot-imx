# (C) Copyright 2015 Freescale Semiconductor, Inc.
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y  := imx6pbc.o pin_mux.o
obj-y  += ../../freescale/common/

CFLAGS_pin_mux.o += -I$(srctree)/board/$(VENDOR)/common

extra-$(CONFIG_USE_IMXIMG_PLUGIN) := plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@

extra-y += ../../../arch/$(ARCH)/dts/imx6dp-chargepoint-pbc.dts
$(obj)/../../../arch/$(ARCH)/dts/imx6dp-chargepoint-pbc.dts: \
		$(src)/imx6dp-chargepoint-pbc.dts
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi
$(obj)/../../../arch/$(ARCH)/dts/chargepoint-pubkey.dtsi: \
		$(src)/../common/chargepoint-pubkey.dtsi
	ln -fsn $(abspath $<) $@

extra-y += ../../../arch/$(ARCH)/dts/prod-pubkey-pbc.dtsi
$(obj)/../../../arch/$(ARCH)/dts/prod-pubkey-pbc.dtsi: \
		$(src)/prod-pubkey-pbc.dtsi
	ln -fsn $(abspath $<) $@
