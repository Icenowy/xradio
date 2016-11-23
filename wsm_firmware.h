#ifndef __XRADIO_WSM_FIRMWARE_H
#define __XRADIO_WSM_FIRMWARE_H

#include "xr819.h"

int wsm_handle_exception(struct xr819* priv, u8 * data, size_t len);

#endif
