/*
 *  TI Bluesleep driver
 *	Kernel module responsible for Wake up of Host
 *  Copyright (C) 2009-2010 Texas Instruments


 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * Copyright (C) 2006-2007 - Motorola
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 *  Date         Author           Comment
 * -----------  --------------   --------------------------------
 * 2006-Apr-28  Motorola         The kernel module for running the Bluetooth(R)
 *                               Sleep-Mode Protocol from the Host side
 * 2006-Sep-08  Motorola         Added workqueue for handling sleep work.
 * 2007-Jan-24  Motorola         Added mbm_handle_ioi() call to ISR.
 * 2009-Aug-10  Motorola         Changed "add_timer" to "mod_timer" to solve
 *                               race when flurry of queued work comes in.
*/

#include <linux/module.h>       /* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/param.h>
#include <linux/bitops.h>
#include <linux/termios.h>
#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <linux/serial_core.h>
#include <linux/tegra_uart.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h> /* event notifications */
#include "hci_uart.h"

/*
 * Defines
 */

#define VERSION	 "1.1"

#define POLARITY_LOW 0
#define POLARITY_HIGH 1

struct bluesleep_info {
	unsigned host_wake_irq;
	struct uart_port *uport;
	struct wake_lock wake_lock;
	int irq_polarity;
	int has_ext_wake;
};


/* state variable names and bit positions */
#define FLAG_RESET      	0x00
#define BT_ACTIVE		0x02
#define BT_SUSPEND		0x04


/* work function */
static void hostwake_sleep_work(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(ti_sleep_workqueue, hostwake_sleep_work);

/* Macros for handling sleep work */
#define hostwake_workqueue()     schedule_delayed_work(&ti_sleep_workqueue, 0)

static struct bluesleep_info *bsi;

/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);

/*
 * Global variables
 */
/** Global state flags */
static unsigned long flags;

/** Tasklet to respond to change in hostwake line */
static struct tasklet_struct hostwake_task;

/** Lock for state transitions */
static spinlock_t rw_lock;

/*
 * Local functions
 */
static void hsuart_power(int on)
{
	pr_debug("%s", __func__);

	if (on) {
		tegra_uart_request_clock_on(bsi->uport);
		tegra_uart_set_mctrl(bsi->uport, TIOCM_RTS);
	} else {
		tegra_uart_set_mctrl(bsi->uport, 0);
		tegra_uart_request_clock_off(bsi->uport);
	}
}



/**
 * @brief@  main sleep work handling function which update the flags
 * and activate and deactivate UART .
 */

static void hostwake_sleep_work(struct work_struct *work)
{
	pr_debug("%s", __func__);
	free_irq(bsi->host_wake_irq, "tibluesleep");
	/*Activating UART */
	if (test_bit(BT_SUSPEND, &flags)) {
		BT_DBG("Activate UART");
		hsuart_power(1);

	}
	bsi->has_ext_wake = 0;
	clear_bit(BT_SUSPEND, &flags);
	set_bit(BT_ACTIVE, &flags);

}


/**
 * A tasklet function that runs in tasklet context
 * @param data Not used.
 */
static void bluesleep_hostwake_task(unsigned long data)
{
	pr_debug("%s", __func__);
	disable_irq(bsi->host_wake_irq);
	spin_lock(&rw_lock);
	hostwake_workqueue();
	spin_unlock(&rw_lock);
}


/**
 * Schedules a tasklet to run when receiving an interrupt on the
 * <code>HOST_WAKE</code> GPIO pin.
 * @param irq Not used.
 * @param dev_id Not used.
 */
static irqreturn_t bluesleep_hostwake_isr(int irq, void *dev_id)
{

	pr_debug("%s", __func__);
	/* schedule a tasklet to handle the change in the host wake line */
	bsi->has_ext_wake = 1;
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}

/**
 * Starts the Sleep-Mode Protocol on the Host.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */

  int bluesleep_start(struct uart_port *uport)
{
	int retval;
	bsi->uport = uport;
	pr_debug("%s", __func__);

	if (test_bit(BT_SUSPEND, &flags)) {
		BT_DBG("bluesleep_acquire irq");
		if (bsi->irq_polarity == POLARITY_LOW) {
			retval = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
					IRQF_DISABLED | IRQF_TRIGGER_FALLING,
					"bluetooth hostwake", "tibluesleep");
		} else 	{
			retval = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
					IRQF_DISABLED | IRQF_TRIGGER_RISING,
					"bluetooth hostwake", "tibluesleep");
		}
		if (retval  < 0) {
			BT_ERR("Couldn't acquire BT_HOST_WAKE IRQ");
			goto fail;
		}

		retval = enable_irq_wake(bsi->host_wake_irq);
		if (retval < 0) {
			BT_ERR("Couldn't enable BT_HOST_WAKE as wakeup"
					"interrupt retval %d\n", retval);
			free_irq(bsi->host_wake_irq, NULL);
			goto fail;
		}
	}

	return 0;
fail:
	atomic_inc(&open_count);
	return retval;
}

/**
 * Stops the Sleep-Mode Protocol on the Host.
 */
  void bluesleep_stop(void)
{

	pr_debug("%s", __func__);

	if (disable_irq_wake(bsi->host_wake_irq))
		BT_ERR("Couldn't disable hostwake IRQ wakeup mode\n");

	free_irq(bsi->host_wake_irq, NULL);
}

static int bluesleep_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;

	bsi = kzalloc(sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						"host_wake");
	if (!res) {
		BT_ERR("couldn't find host_wake irq\n");
		ret = -ENODEV;
		goto free_bsi;
	}

	bsi->host_wake_irq = res->start;

	if (bsi->host_wake_irq < 0) {
		BT_ERR("couldn't find host_wake irq");
		ret = -ENODEV;
		goto free_bsi;
	}
	if (res->flags & IORESOURCE_IRQ_LOWEDGE)
		bsi->irq_polarity = POLARITY_LOW;/*low edge (falling edge)*/
	else
		bsi->irq_polarity = POLARITY_HIGH;/*anything else*/

	wake_lock_init(&bsi->wake_lock, WAKE_LOCK_SUSPEND, "bluesleep");
	clear_bit(BT_SUSPEND, &flags);
	set_bit(BT_ACTIVE, &flags);

	return 0;

free_bsi:
	kfree(bsi);
	return ret;
}

static int bluesleep_remove(struct platform_device *pdev)
{
	pr_debug("%s", __func__);
	kfree(bsi);
	return 0;
}


static int bluesleep_resume(struct platform_device *pdev)
{

	pr_debug("%s", __func__);
	if (test_bit(BT_SUSPEND, &flags)) {

		if ((bsi->uport != NULL) && (bsi->has_ext_wake)) {
			tegra_uart_request_clock_on(bsi->uport);
			tegra_uart_set_mctrl(bsi->uport, TIOCM_RTS);
		}
		clear_bit(BT_SUSPEND, &flags);
		set_bit(BT_ACTIVE, &flags);
	}

	return 0;
}

static int bluesleep_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_debug("%s", __func__);
	set_bit(BT_SUSPEND, &flags);
	return 0;
}

static struct platform_driver bluesleep_driver = {
	.probe = bluesleep_probe,
	.remove = bluesleep_remove,
	.suspend = bluesleep_suspend,
	.resume = bluesleep_resume,
	.driver = {
		.name = "tibluesleep",
		.owner = THIS_MODULE,
	},
};
/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __init bluesleep_init(void)
{
	int retval;

	BT_INFO("BlueSleep Mode Driver Ver %s", VERSION);

	retval = platform_driver_register(&bluesleep_driver);
	if (retval)
		goto fail;

	if (bsi == NULL)
		return 0;

	flags = FLAG_RESET; /* clear all status bits */

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

	/* initialize host wake tasklet */
	tasklet_init(&hostwake_task, bluesleep_hostwake_task, 0);

	return 0;
fail:
	return retval;
}

/**
 * Cleans up the module.
 */

static void __exit bluesleep_exit(void)
{
	if (bsi == NULL)
		return;
	/* assert bt wake */
	free_irq(bsi->host_wake_irq, NULL);
	platform_driver_unregister(&bluesleep_driver);

}

module_init(bluesleep_init);
module_exit(bluesleep_exit);

MODULE_DESCRIPTION("TI Bluetooth Sleep Mode Driver ver %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
