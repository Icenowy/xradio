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
/*
 * sbus priv forward definition.
 * Implemented and instantiated in particular modules.
 */

struct xradio_common;
/*sdio bus private struct*/
#define SDIO_UNLOAD   0
#define SDIO_LOAD     1

typedef void (*sbus_irq_handler)(void *priv);

struct sdio_priv {
	struct sdio_func     *func;
	spinlock_t            lock;
	sbus_irq_handler      irq_handler;
	void                 *irq_priv;
};


//sbus init functions
int sbus_sdio_init(void);
void  sbus_sdio_deinit(void);

#endif /* __SDIO_H */
