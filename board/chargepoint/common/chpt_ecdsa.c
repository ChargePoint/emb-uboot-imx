/*
 * ChargePoint driver implementation for ECDSA verification
 * using libcrypto (no hardware specifics assumed)
 * bobwolff-cpi - Jan 9, 2024
*/

#include <stdio.h>
#include <stdint.h>

#include <tinycrypt/ecc.h>
#include <tinycrypt/ecc_platform_specific.h>
#include <tinycrypt/ecc_dsa.h>
#include <tinycrypt/ecc_dh.h>
#include <tinycrypt/constants.h>
#include <tinycrypt/sha256.h>

#include <crypto/ecdsa-uclass.h>
#include <dm/uclass.h>
#include <u-boot/ecdsa.h>

#include <malloc.h>

int chpt_ecdsa_verify(struct udevice *dev, const struct ecdsa_public_key *pubkey,
		      const void *hash, size_t hash_len,
		      const void *signature, size_t sig_len)
{
	uint8_t *pubkey_bits=NULL;

	if (!pubkey || !hash || !hash_len || !signature || !sig_len) {
		return -EINVAL;
	}

	pubkey_bits = (uint8_t*)malloc(2*pubkey->size_bits/8);
	if (!pubkey_bits) {
		printf("Failed allocation via malloc for public key.\n");
		return -ENOMEM;
	}

	memcpy(pubkey_bits, pubkey->x, pubkey->size_bits/8);
	memcpy(pubkey_bits + pubkey->size_bits/8, pubkey->y, 2*pubkey->size_bits/8);

	int success = uECC_verify((const uint8_t*)pubkey_bits, (const uint8_t*)hash,
		(unsigned int)hash_len, (const uint8_t*)signature, uECC_secp256r1());

//printf("%s: before free() operation.\n", __func__);
//	free(pubkey_bits);
//printf("%s: before returning.\n", __func__);
	return success==1 ? 0 : -ENOMSG;
}

static const struct ecdsa_ops ecdsa_ops = {
        .verify      = chpt_ecdsa_verify,
};

static const struct udevice_id ecdsa_ids[] = {
        { .compatible = "chpt,ecdsa256" },
        { }
};

U_BOOT_DRIVER(chpt_ecdsa256) = {
        .name   = "chpt_ecdsa",
        .id     = UCLASS_ECDSA,
        .of_match = ecdsa_ids,
        .ops    = &ecdsa_ops,
};
