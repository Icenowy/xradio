#include "wsm_buf.h"
#include "bh.h"

void wsm_buf_init(struct wsm_buf *buf) {
	int size = (SDIO_BLOCK_SIZE << 1); //for sdd file big than SDIO_BLOCK_SIZE
	buf->begin = kmalloc(size, GFP_KERNEL);
	buf->end = buf->begin ? &buf->begin[size] : buf->begin;
	wsm_buf_reset(buf);
}

int wsm_init(struct xr819* priv) {
	mutex_init(&priv->wsm.wsm_cmd_mux);
	wsm_buf_init(&priv->wsm.wsm_cmd_buf);
	spin_lock_init(&priv->wsm.wsm_cmd.lock);
	init_waitqueue_head(&priv->wsm.wsm_cmd_wq);

	init_waitqueue_head(&priv->wsm.wsm_startup_done);
	priv->wsm.caps.firmwareReady = 0;
}

int wsm_buf_reserve(struct wsm_buf *buf, size_t extra_size) {
	size_t pos = buf->data - buf->begin;
	size_t size = pos + extra_size;

	if (size & (SDIO_BLOCK_SIZE - 1)) {
		size &= SDIO_BLOCK_SIZE;
		size += SDIO_BLOCK_SIZE;
	}

	buf->begin = krealloc(buf->begin, size, GFP_KERNEL);
	if (buf->begin) {
		buf->data = &buf->begin[pos];
		buf->end = &buf->begin[size];
		return 0;
	} else {
		buf->end = buf->data = buf->begin;
		return -ENOMEM;
	}
}

void wsm_buf_reset(struct wsm_buf *buf) {
	if (buf->begin) {
		buf->data = &buf->begin[4];
		*(u32 *) buf->begin = 0;
	} else
		buf->data = buf->begin;
}
