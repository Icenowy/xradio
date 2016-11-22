/*
 * Data Transmission thread for XRadio drivers
 *
 * Copyright (c) 2013, XRadio
 * Author: XRadio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef XRADIO_BH_H
#define XRADIO_BH_H

#define XRADIO_BH_THREAD   "xradio_bh"

/* extern */struct xradio_common;

#define SDIO_BLOCK_SIZE (528)

int xradio_register_bh(struct xr819 *priv);
void xradio_unregister_bh(struct xr819 *priv);
void xradio_irq_handler(struct xr819 *priv);
void xradio_bh_wakeup(struct xr819 *priv);
int xradio_bh_suspend(struct xr819 *priv);
int xradio_bh_resume(struct xr819 *hw_priv);

/* Must be called from BH thread. */
void xradio_enable_powersave(struct xradio_vif *priv, bool enable);
int wsm_release_tx_buffer(struct xr819 *hw_priv, int count);
int wsm_release_vif_tx_buffer(struct xr819 *hw_priv, int if_id, int count);
int xradio_init_resv_skb(struct xr819 *hw_priv);
void xradio_deinit_resv_skb(struct xr819 *hw_priv);
int xradio_realloc_resv_skb(struct xr819 *hw_priv, struct sk_buff *skb);
#endif /* XRADIO_BH_H */
