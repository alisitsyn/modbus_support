/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdbool.h>

// The workaround to statically link whole test library
__attribute__((unused)) bool mb_patch_lib_include = true;