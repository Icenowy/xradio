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

struct xr819_firmware {
#ifdef USE_VFS_FIRMWARE
	const struct xr_file *sdd;
#else
	const struct firmware *sdd;
#endif
	u8 conf_listen_interval;
	bool is_BT_Present;
};

struct xr819_hardware {
	int hw_type;
	int hw_revision;
	int fw_revision;
};

struct xr819_bh {
	atomic_t rx;
	atomic_t tx;
	atomic_t term;
	atomic_t suspend;
	struct task_struct *thread;
	int error;
#ifdef BH_USE_SEMAPHORE
	struct semaphore sem;
	atomic_t wk;
#else
	wait_queue_head_t wq;
#endif
	wait_queue_head_t evt_wq;
};

struct sdio_priv {
	struct sdio_func *func;
	spinlock_t lock;
	sbus_irq_handler irq_handler;
	void *irq_priv;

	struct xr819_hardware hardware;
	struct xr819_firmware firmware;
	struct xr819_bh	bh;


	int				buf_id_tx;	/* byte */
	int				buf_id_rx;	/* byte */

};

//sbus init functions
int sbus_sdio_init(void);
void sbus_sdio_deinit(void);

size_t sdio_align_len(struct sdio_priv *self, size_t size);

#endif /* __SDIO_H */
