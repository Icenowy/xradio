CONFIG_XRADIO := m
CONFIG_XRADIO_SDIO := y
CONFIG_XRADIO_USE_EXTENSIONS := y

#	txrx.o \
# \
#	ap.o \
#	scan.o \
#	queue.o \
#	debug.o \

xradio_wlan-y := \
	main.o \
	sdio.o \
	fwio.o \
	bh.o \
	wsm.o \
	wsm_buf.o \
	wsm_command.o \
	wsm_confirm.o \
	wsm_firmware.o \
	mac80211.o \
	txrx.o \
	sta.o \
	ap.o

#xradio_wlan-$(CONFIG_PM)		+= pm.o
xradio_wlan-$(CONFIG_XRADIO_ITP)	+= itp.o

ccflags-y += -DP2P_MULTIVIF
ccflags-y += -DMCAST_FWDING
ccflags-y += -DXRADIO_SUSPEND_RESUME_FILTER_ENABLE
ccflags-y += -DAP_AGGREGATE_FW_FIX
ccflags-y += -DAP_HT_CAP_UPDATE
ccflags-y += -DHW_RESTART
ccflags-y += -DHW_ERROR_WIFI_RESET
ccflags-y += -DAP_HT_COMPAT_FIX
ccflags-y += -DCONFIG_XRADIO_DUMP_ON_ERROR
ccflags-y += -DCONFIG_XRADIO_NON_POWER_OF_TWO_BLOCKSIZES

ccflags-y += -DCONFIG_XRADIO_SUSPEND_POWER_OFF

# Extra IE for probe response from upper layer is needed in P2P GO
# For offloading probe response to FW, the extra IE must be included
# in the probe response template
ccflags-y += -DPROBE_RESP_EXTRA_IE
ccflags-y += -DIPV6_FILTERING

# Modified by wzw
ccflags-y += -DTES_P2P_0002_ROC_RESTART
ccflags-y += -DTES_P2P_000B_EXTEND_INACTIVITY_CNT
ccflags-y += -DTES_P2P_000B_DISABLE_EAPOL_FILTER
ccflags-y += -DXRADIO_USE_LONG_DTIM_PERIOD
ccflags-y += -DXRADIO_USE_LONG_KEEP_ALIVE_PERIOD

# Mac addr config, disable hex for default.
#ccflags-y += -DXRADIO_MACPARAM_HEX

# Mac addr generate from chipid.
#ccflags-y += -DXRADIO_MACADDR_FROM_CHIPID

# Use semaphore to sync bh txrx.
#ccflags-y += -DBH_USE_SEMAPHORE

obj-$(CONFIG_XRADIO) += xradio_wlan.o

