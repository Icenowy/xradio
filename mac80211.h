#ifndef __XRADIO_NETIF_H
#define __XRADIO_NETIF_H

#include "xr819.h"

int netif_init(struct device* dev, struct xr819** priv);
int netif_register(struct xr819* priv);

#endif
