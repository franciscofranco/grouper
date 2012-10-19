#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <mach/board-grouper-misc.h>

#include "baseband-xmm-power.h"
#include "board-grouper.h"

#include "pm-irq.h"
#include "ril.h"
#include "ril_modem_crash.h"

#define _ATTR_MODE S_IRUSR | S_IWUSR | S_IRGRP


static struct device *dev;
static struct workqueue_struct *workqueue;
static struct work_struct crash_work;
static struct wake_lock wakelock;
static struct switch_dev crash_sdev;
static int do_crash_dump = 0;
static int do_crash_dump_work = 0;

void ril_change_modem_crash_mode(void)
{
	int value = 0;

	if (do_crash_dump == 1) {
		value = gpio_get_value(MOD_HANG);
		if (value) {
			RIL_INFO("do_crash_dump is on!\n");
			queue_work(workqueue, &crash_work);
		}
	}
}
EXPORT_SYMBOL_GPL(ril_change_modem_crash_mode);

/* sysfs functions */

static ssize_t show_cdump_state(struct device *class,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", do_crash_dump);
}

static ssize_t store_cdump_state(struct device *class, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;

	if (sscanf(buf, "%d", &enable) != 1)
		return -EINVAL;

	if ((enable != 1) && (enable != 0))
		return -EINVAL;

	RIL_INFO("enable: %d\n", enable);

	if (enable)
		do_crash_dump = 1;
	else
		do_crash_dump = 0;

	return strnlen(buf, count);
}

static ssize_t show_crash_mode_state(struct device *class,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", do_crash_dump_work);
}

static ssize_t store_crash_mode_state(struct device *class, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;

	if (sscanf(buf, "%d", &enable) != 1)
		return -EINVAL;

	if ((enable != 1) && (enable != 0))
		return -EINVAL;

	RIL_INFO("enable: %d\n", enable);

	if (enable) {
		ril_change_modem_crash_mode();
	}

	return strnlen(buf, count);
}

static struct device_attribute SYSFS_LIST[] = {
	__ATTR(crash_dump_onoff, _ATTR_MODE, show_cdump_state, store_cdump_state),
	__ATTR(change_crash_mode, _ATTR_MODE, show_crash_mode_state, store_crash_mode_state),
	__ATTR_NULL,
};

/* modem crash switch functions */
static ssize_t crash_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "crash_dump_det");
}

static ssize_t crash_print_state(struct switch_dev *sdev, char *buf)
{
	int state = -1;
	if (switch_get_state(sdev))
		state = 1;
	else
		state = 0;

	return sprintf(buf, "%d\n", state);
}

static int ril_crash_switch_init(void)
{
	int rc = 0;

	/* register switch class */
	crash_sdev.name = "crash_dump_det";
	crash_sdev.print_name = crash_print_name;
	crash_sdev.print_state = crash_print_state;

	rc = switch_dev_register(&crash_sdev);

	if (rc < 0)
		goto failed;

	return 0;

failed:
	return rc;
}

static void ril_crash_switch_exit(void)
{
	switch_dev_unregister(&crash_sdev);
}

static void ril_crash_dump_work(struct work_struct *work)
{
	if (do_crash_dump_work == 0) {
		disable_irq(gpio_to_irq(MOD_HANG));

		wake_lock(&wakelock);

		baseband_usb_hsic_host_register(0);
		msleep(200);
		gpio_set_value(USB_SW_SEL, 1);
		mdelay(5);
		gpio_set_value(MOD_VBUS_ON, 1);
		mdelay(5);
		baseband_usb_utmip_host_register(1);

		switch_set_state(&crash_sdev, 1);
		do_crash_dump_work = 1;

		wake_unlock(&wakelock);
	}
}

/* IRQ Handlers */
irqreturn_t ril_ipc_sus_req_irq(int irq, void *dev_id)
{
	RIL_INFO("The IMC modem crashed!\n");

	ril_change_modem_crash_mode();

	return IRQ_HANDLED;
}

/* init and exit functions */
int ril_modem_crash_init(struct device *target_device, struct workqueue_struct *queue)
{
	int i, err;

	workqueue = queue;
	dev = target_device;

	/* init variable */
	INIT_WORK(&crash_work, ril_crash_dump_work);
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "baseband_xmm_ril");

	// MOD_HANG is not wakeup soruce.
	// Modem will wake AP first.
	err = request_irq(gpio_to_irq(MOD_HANG),
		ril_ipc_sus_req_irq,
		IRQF_TRIGGER_RISING,
		"MOD_HANG",
		NULL);
	if (err < 0) {
		pr_err("%s - request irq MOD_HANG failed\n", __func__);
		goto failed_irq;
	}
	tegra_pm_irq_set_wake_type(gpio_to_irq(MOD_HANG),
		IRQF_TRIGGER_RISING);
	enable_irq_wake(gpio_to_irq(MOD_HANG));

	/* create sysfs */
	for (i = 0; i < (ARRAY_SIZE(SYSFS_LIST) - 1); ++i) {
		err = device_create_file(dev, &SYSFS_LIST[i]);
		if (err < 0) {
			RIL_ERR("%s: create file of [%d] failed, err = %d\n",
					__func__, i, err);
			goto failed_create_sysfs;
		}
	}

	/* register crash dump switch */
	err = ril_crash_switch_init();
	if (err < 0) {
		pr_err("%s - register modem crash switch failed\n",
			__func__);
		goto failed_switch_init;
	}
	RIL_INFO("init successfully\n");
	return 0;

failed_switch_init:
	while (i >= 0) {
		device_remove_file(dev, &SYSFS_LIST[i]);
		--i;
	}
failed_create_sysfs:
	free_irq(gpio_to_irq(MOD_HANG), NULL);
failed_irq:
	return err;
}

void ril_modem_crash_exit(void)
{
	int i;
	ril_crash_switch_exit();
	for (i = 0; i < (ARRAY_SIZE(SYSFS_LIST) - 1); ++i) {
		device_remove_file(dev, &SYSFS_LIST[i]);
	}
	free_irq(gpio_to_irq(MOD_HANG), NULL);
	wake_lock_destroy(&wakelock);
}

