#ifndef _ARCH_ARM_GPIO_H
#define _ARCH_ARM_GPIO_H

#if CONFIG_ARCH_NR_GPIO > 0
#define ARCH_NR_GPIO CONFIG_ARCH_NR_GPIO
#endif

/* not all ARM platforms necessarily support this API ... */
#include <mach/gpio.h>

#endif /* _ARCH_ARM_GPIO_H */
