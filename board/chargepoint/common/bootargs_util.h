/*
 * Copyright 2022 ChargePoint, Inc.
 *
 * Helper function to update bootargs environment variable.
 *
 * This particular feature is for ChargePoint.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _BOOTARGS_UTIL_H_
#define _BOOTARGS_UTIL_H_

/*
 * bootargs_append_param() - append parameter to bootargs environment variable
 *
 * @param:   Pointer to null-terminated parameter name. Must be valid pointer.
 * @value:   Pointer to null-terminated parameter value. If not NULL, appended
 *           string is in form 'param=value'. If NULL, appended string is 
 *           'param' without trailing equal sign.
 * @return:  none
 */
void bootargs_append_param(const char *param, const char *value);

#endif /* _BOOTARGS_UTIL_H_ */
