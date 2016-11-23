#include "wsm_firmware.h"
#include "wsm_putget.h"

static const char * const reason_str[] = { "undefined instruction",
		"prefetch abort", "data abort", "unknown error", };

int wsm_handle_exception(struct xr819 *hw_priv, u8 *data, size_t len) {
	struct wsm_buf buf;
	u32 reason;
	u32 reg[18];
	char fname[48];
	int i = 0;
#if defined(CONFIG_XRADIO_USE_EXTENSIONS)
	struct xradio_vif *priv = NULL;
#endif

#if defined(CONFIG_XRADIO_USE_EXTENSIONS)
	/* Send the event upwards on the FW exception */
	xradio_pm_stay_awake(&hw_priv->pm_state, 3*HZ);

	spin_lock(&hw_priv->vif_list_lock);
	xradio_for_each_vif(hw_priv, priv, i) {
		if (!priv)
		continue;
		//ieee80211_driver_hang_notify(priv->vif, GFP_KERNEL);
	}
	spin_unlock(&hw_priv->vif_list_lock);
#endif

	buf.begin = buf.data = data;
	buf.end = &buf.begin[len];

	reason = WSM_GET32(&buf);
	for (i = 0; i < ARRAY_SIZE(reg); ++i)
		reg[i] = WSM_GET32(&buf);
	WSM_GET(&buf, fname, sizeof(fname));

	if (reason < 4) {
		dev_err(hw_priv->dev, "Firmware exception: %s.\n", reason_str[reason]);
	} else {
		dev_err(hw_priv->dev, "Firmware assert at %.*s, line %d, reason=0x%x\n",
				sizeof(fname), fname, reg[1], reg[2]);
	}

	for (i = 0; i < 12; i += 4) {
		dev_err(hw_priv->dev, "Firmware:"
				"R%d: 0x%.8X, R%d: 0x%.8X, R%d: 0x%.8X, R%d: 0x%.8X,\n", i + 0,
				reg[i + 0], i + 1, reg[i + 1], i + 2, reg[i + 2], i + 3,
				reg[i + 3]);
	}
	dev_err(hw_priv->dev, "Firmware:"
			"R12: 0x%.8X, SP: 0x%.8X, LR: 0x%.8X, PC: 0x%.8X,\n", reg[i + 0],
			reg[i + 1], reg[i + 2], reg[i + 3]);
	i += 4;
	dev_err(hw_priv->dev, "Firmware:CPSR: 0x%.8X, SPSR: 0x%.8X\n", reg[i + 0],
			reg[i + 1]);

	return 0;

	underflow: dev_err(hw_priv->dev, "Firmware exception.\n");
	print_hex_dump_bytes("Exception: ", DUMP_PREFIX_NONE, data, len);
	return -EINVAL;
}
