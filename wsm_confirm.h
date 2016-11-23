#ifndef __XRADIO_WSM_CONFIRM_H
#define __XRADIO_WSM_CONFIRM_H

#include "xr819.h"
#include "wsm.h"

int wsm_write_mib_confirm(struct xr819 *hw_priv, struct wsm_mib *arg,
		struct wsm_buf *buf, int interface_link_id);
int wsm_configuration_confirm(struct xr819 *hw_priv,
		struct wsm_configuration *arg, struct wsm_buf *buf);

#endif
