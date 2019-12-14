# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
#
# (C) Copyright 2011 Freescale Semiconductor, Inc.

obj-y  := mx6sabresd_pbc.o pin_mux.o

extra-$(CONFIG_USE_IMXIMG_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
