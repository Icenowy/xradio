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


#ifndef __SBUS_H
#define __SBUS_H

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
struct sbus_priv {
	struct sdio_func     *func;
	spinlock_t            lock;
	sbus_irq_handler      irq_handler;
	void                 *irq_priv;
	wait_queue_head_t     init_wq;
	int                   load_state;
};

//sbus init functions
struct device * sbus_sdio_init(struct sbus_priv **sdio_priv);
void  sbus_sdio_deinit(void);

#endif /* __SBUS_H */
