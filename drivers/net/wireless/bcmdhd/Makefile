# bcmdhd
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
	-DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DBDC -DTOE                         \
	-DDHD_BCMEVENTS -DDONGLEOVERLAYS                                      \
	-DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS                              \
	-DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
	-DPKT_FILTER_SUPPORT                                                  \
	-DPNO_SUPPORT                                                         \
	-Idrivers/net/wireless/bcmdhd -Idrivers/net/wireless/bcmdhd/include

# Pre-init chip when driver starting.
DHDCFLAGS += -DEMBEDDED_PLATFORM
# Platform driver enabled for GPIO control, card detect and getting information of Wi-Fi chip
DHDCFLAGS += -DCUSTOMER_HW2
# Support the sdio_reset_comm(). Quick back to live when system resuming.
DHDCFLAGS += -DMMC_SDIO_ABORT
# Send out the null packet in period to keep the connection with AP
DHDCFLAGS += -DKEEP_ALIVE
# Debug flags
DHDCFLAGS += -DBCMDBG -DSDTEST -DSHOW_EVENTS -DDHD_DEBUG
# For ICS build-in mode without module removing anymore
DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD 

ifeq ($(CONFIG_BCMDHD_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC
else
DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=2
endif

# Use HW_OOB as default. HW_OOB is level-trigger
ifeq ($(CONFIG_BCMDHD_HW_OOB),y)
DHDCFLAGS += -DHW_OOB -DOOB_INTR_ONLY
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o bcmwifi.o dhd_sdio.o bcmevent.o dhd_bta.o hndpmu.o  \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o

obj-$(CONFIG_BCMDHD) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DCSCAN
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o dhd_cfg80211.o wl_cfgp2p.o wl_linux_mon.o
DHDCFLAGS += -DWL_CFG80211
# P2P support
DHDCFLAGS += -DWLP2P
# WAPI support
#DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI
endif

EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
DHDCFLAGS += -I$(M) -I$(M)/include
EXTRA_LDFLAGS += --strip-debug
endif
