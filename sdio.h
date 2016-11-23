/*
 * Sbus interfaces for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SDIO_H
#define __SDIO_H

#include <linux/version.h>
#include <linux/module.h>

#include "xr819.h"

//sbus init functions
int sbus_sdio_init(void);
void sbus_sdio_deinit(void);

size_t sdio_align_len(struct xr819 *self, size_t size);

#endif /* __SDIO_H */
