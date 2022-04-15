/*
 * Copyright 2022 ChargePoint, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <stdlib.h>

#include "bootargs_util.h"

void bootargs_append_param(const char *param, const char *value)
{
    size_t value_len;
    size_t param_len;
    size_t old_bootargs_len;
    size_t new_bootargs_len;
    const char *old_bootargs = env_get("bootargs");
    char *new_bootargs;
    char *p;

    if (old_bootargs) {
        old_bootargs_len = strlen(old_bootargs);
    }

    param_len = strlen(param);

    if (value) {
        value_len = strlen(value);
    }

    new_bootargs_len = param_len + 1 /* null char */;

    if (old_bootargs) {
        new_bootargs_len += old_bootargs_len + 1 /* space */;
    }

    if (value) {
        new_bootargs_len += value_len + 1 /* equal sign */;
    }

    new_bootargs = malloc(new_bootargs_len);
    if (new_bootargs == NULL) {
        printf("%s malloc failed\n", __FUNCTION__);
        return;
    }

    p = new_bootargs;

    if (old_bootargs) {
        memcpy(p, old_bootargs, old_bootargs_len);
        p += old_bootargs_len;
        *p++ = ' ';
    }

    memcpy(p, param, param_len);
    p += param_len;

    if (value) {
        *p++ = '=';
        memcpy(p, value, value_len);
        p += value_len;
    }

    *p = '\0';

    env_set("bootargs", new_bootargs);

    free(new_bootargs);
}
