# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2000-2007
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.

obj-y += \
	source/ecc.o \
	source/ecc_dsa.o \
	source/sha256.o \
	source/aes_decrypt.o \
	source/aes_encrypt.o \
	source/cbc_mode.o \
	source/ctr_mode.o \
	source/ctr_prng.o \
	source/hmac.o \
	source/hmac_prng.o \
	source/ecc_dh.o \
	source/ccm_mode.o \
	source/cmac_mode.o \
	source/utils.o

ccflags-y := -I$(srctree)/lib/tinycrypt/include
