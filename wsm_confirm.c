#include "wsm_confirm.h"
#include "wsm.h"
#include "wsm_putget.h"

static int wsm_generic_confirm(struct xr819 *hw_priv, void *arg,
		struct wsm_buf *buf) {
	u32 status = WSM_GET32(buf);
	if (status != WSM_STATUS_SUCCESS) {
		dev_err(hw_priv->dev, "negative command confirmation\n");
		return -EINVAL;
	}
	return 0;
	underflow: return -EINVAL;
}

int wsm_write_mib_confirm(struct xr819 *hw_priv, struct wsm_mib *arg,
		struct wsm_buf *buf, int interface_link_id) {
	int ret;
	int i;
	struct xradio_vif *priv;
	ret = wsm_generic_confirm(hw_priv, arg, buf);
	if (ret)
		return ret;

	/*wsm_set_operational_mode confirm.*/
	if (arg->mibId == 0x1006) {
#if 0
		const char *p = arg->buf;
		bool powersave_enabled = (p[0] & 0x0F) ? true : false;

		/* update vif PM status. */
		priv = xrwl_hwpriv_to_vifpriv(hw_priv, interface_link_id);
		if (priv) {
			xradio_enable_powersave(priv, powersave_enabled);
			spin_unlock(&priv->vif_lock);
		}

		/* HW powersave base on vif except for generic vif. */
		spin_lock(&hw_priv->vif_list_lock);
		xradio_for_each_vif(hw_priv, priv, i)
		{
#ifdef P2P_MULTIVIF
			if ((i == (XRWL_MAX_VIFS - 1)) || !priv)
#else
			if (!priv)
#endif
			continue;
			powersave_enabled &= !!priv->powersave_enabled;
		}
		hw_priv->powersave_enabled = powersave_enabled;
		spin_unlock(&hw_priv->vif_list_lock);
#endif
	}
	return 0;
}

int wsm_configuration_confirm(struct xr819 *hw_priv,
		struct wsm_configuration *arg, struct wsm_buf *buf) {
	int ret, i;

	ret = wsm_generic_confirm(hw_priv, arg, buf);
	if (ret)
		return ret;

	WSM_GET(buf, arg->dot11StationId, ETH_ALEN);
	arg->dot11FrequencyBandsSupported = WSM_GET8(buf);
	WSM_SKIP(buf, 1);
	arg->supportedRateMask = WSM_GET32(buf);
	for (i = 0; i < 2; ++i) {
		arg->txPowerRange[i].min_power_level = WSM_GET32(buf);
		arg->txPowerRange[i].max_power_level = WSM_GET32(buf);
		arg->txPowerRange[i].stepping = WSM_GET32(buf);
	}
	return ret;

	underflow: return -EINVAL;
}
