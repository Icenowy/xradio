#ifndef __XRADIO_SDIO_H
#define __XRADIO_SDIO_H

int sdio_irq_subscribe(struct sbus_priv *self, sbus_irq_handler handler,
		void *priv);
int sdio_irq_unsubscribe(struct sbus_priv *self);
size_t sdio_align_len(struct sbus_priv *self, size_t size);
void sdio_lock(struct sbus_priv *self);
void sdio_unlock(struct sbus_priv *self);
int sdio_set_blk_size(struct sbus_priv *self, size_t size);
int sdio_data_read(struct sbus_priv *self, unsigned int addr, void *dst,
		int count);
int sdio_data_write(struct sbus_priv *self, unsigned int addr, const void *src,
		int count);
int sdio_pm(struct sbus_priv *self, bool  suspend);

#endif
