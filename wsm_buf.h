#ifndef __XRADIO_WSM_BUFF_H
#define __XRADIO_WSM_BUFF_H

#include "xr819.h"

void wsm_buf_init(struct wsm_buf *buf);
int wsm_init(struct xr819* priv);
int wsm_buf_reserve(struct wsm_buf *buf, size_t extra_size);
void wsm_buf_reset(struct wsm_buf *buf);

#endif
