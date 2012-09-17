obj-$(CONFIG_SENSORS_CAP1106)           += cap1106.o

ifeq ($(CONFIG_RIL), y)
EXTRA_CFLAGS = -DCONFIG_RIL -Iarch/arm/mach-tegra/
endif
