#ifndef RIL_H
#define RIL_H

#include "gpio-names.h"

/* DEBUG */
#define RIL_DEBUG 1

#if RIL_DEBUG
#define RIL_INFO(format, arg...) \
	printk(KERN_INFO "RIL: [%s] " format , __FUNCTION__ , ## arg)
#else
#define RIL_INFO(format, arg...)
#endif

#define RIL_ERR(format, arg...) \
	printk(KERN_ERR "RIL: [%s] " format , __FUNCTION__ , ## arg)

/* GPIOs */
#define MOD_VBUS_ON     TEGRA_GPIO_PD2
#define USB_SW_SEL      TEGRA_GPIO_PP1
#define SIM_CARD_DET    TEGRA_GPIO_PW3
#define MOD_HANG        TEGRA_GPIO_PN2

#endif
