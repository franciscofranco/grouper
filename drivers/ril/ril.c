#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <mach/board-grouper-misc.h>

#include "pm-irq.h"
#include "ril.h"
#include "ril_sim.h"
#include "ril_modem_crash.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

//**** external symbols


//**** constants

#define _ATTR_MODE S_IRUSR | S_IWUSR | S_IRGRP


//**** local variable declaration

static struct workqueue_struct *workqueue;
static struct device *dev;
static struct class *ril_class;
static dev_t ril_dev;
static int ril_major = 0;
static int ril_minor = 0;
static u32 project_id = 0;

static struct gpio ril_gpios_nakasi3g[] = {
	{ MOD_VBUS_ON,    GPIOF_OUT_INIT_LOW,  "BB_VBUS"    },
	{ USB_SW_SEL,     GPIOF_OUT_INIT_LOW,  "BB_SW_SEL"  },
	{ SIM_CARD_DET,   GPIOF_IN,            "BB_SIM_DET" },
    { MOD_HANG,       GPIOF_IN,            "BB_MOD_HANG"   },
};

//**** IRQ event handler

irqreturn_t ril_ipc_sim_det_irq(int irq, void *dev_id)
{
	return sim_interrupt_handle(irq, dev_id);
}

//**** sysfs callback functions
static int store_gpio(size_t count, const char *buf, int gpio, char *gpio_name)
{
	int enable;

	if (sscanf(buf, "%u", &enable) != 1)
		return -EINVAL;

	if ((enable != 1) && (enable != 0))
		return -EINVAL;

	gpio_set_value(gpio, enable);
	RIL_INFO("Set %s to %d\n", gpio_name, enable);
	return count;
}

/* sysfs functions */
static ssize_t show_vbus_state(struct device *class,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(MOD_VBUS_ON));
}

static ssize_t store_vbus_state(struct device *class, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return store_gpio(count, buf, MOD_VBUS_ON, "MOD_VBUS_ON");
}

//**** sysfs list
static struct device_attribute device_attr_nakasi3g[] = {
	__ATTR(bb_vbus, _ATTR_MODE, show_vbus_state, store_vbus_state),
	__ATTR_NULL,
};

//**** initialize and finalize

static int create_ril_files(void)
{
	int rc = 0, sysfs_cnt = 0;

	rc = alloc_chrdev_region(&ril_dev, ril_minor, 1, "ril");
	ril_major = MAJOR(ril_dev);
	if (rc < 0) {
		RIL_ERR("allocate char device failed\n");
		goto failed;
	}
	RIL_INFO("rc = %d, ril_major = %d\n", rc, ril_major);

	ril_class = class_create(THIS_MODULE, "ril");
	if (IS_ERR(ril_class)) {
		RIL_ERR("ril_class create fail\n");
		rc = PTR_ERR(ril_class);
		goto create_class_failed;
	}

	dev = device_create(ril_class, NULL, MKDEV(ril_major, ril_minor),
			NULL, "files");
	if (IS_ERR(dev)) {
		RIL_ERR("dev create fail\n");
		rc = PTR_ERR(dev);
		goto create_device_failed;
	}

	for (sysfs_cnt = 0; sysfs_cnt < (ARRAY_SIZE(device_attr_nakasi3g) - 1); sysfs_cnt++) {
		rc = device_create_file(dev, &device_attr_nakasi3g[sysfs_cnt]);
		if (rc < 0) {
			RIL_ERR("create file of [%d] failed, err = %d\n", sysfs_cnt, rc);
			goto create_files_failed;
		}
	}

	RIL_INFO("add_ril_dev success\n");
	return 0;

create_files_failed:
	while (sysfs_cnt--)
		device_remove_file(dev, &device_attr_nakasi3g[sysfs_cnt]);
create_device_failed:
	class_destroy(ril_class);
create_class_failed:
	unregister_chrdev_region(ril_dev, 1);
failed:
	return rc;
}

static void remove_ril_files(void)
{
	int i;
	for (i = 0; i < (ARRAY_SIZE(device_attr_nakasi3g) - 1); i++)
		device_remove_file(dev, &device_attr_nakasi3g[i]);
	device_destroy(ril_class, MKDEV(ril_major, ril_minor));
	class_destroy(ril_class);
	unregister_chrdev_region(ril_dev, 1);
}

static int __init ril_init(void)
{
	int err, i;
	RIL_INFO("RIL init\n");

	project_id = grouper_get_project_id();

	/* enable and request gpio(s) */
	if (project_id == GROUPER_PROJECT_NAKASI_3G) {
		RIL_INFO("project_id = NAKASI_3G\n");
		for (i = 0; i < ARRAY_SIZE(ril_gpios_nakasi3g); i++) {
			tegra_gpio_enable(ril_gpios_nakasi3g[i].gpio);
		}
		err = gpio_request_array(ril_gpios_nakasi3g,
				ARRAY_SIZE(ril_gpios_nakasi3g));
		if (err < 0) {
			pr_err("%s - request gpio(s) failed\n", __func__);
			return err;
		}
	} else {
		RIL_ERR("Ril driver doesn't support this project\n");
		return -1;
	}

	/* create device file(s) */
	err = create_ril_files();
	if (err < 0)
		goto failed1;

	/* request ril irq(s) */
	err = request_irq(gpio_to_irq(SIM_CARD_DET),
		ril_ipc_sim_det_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"IPC_SIM_CARD_DET",
		NULL);
	if (err < 0) {
		pr_err("%s - request irq IPC_SIM_CARD_DET failed\n",
			__func__);
		goto failed2;
	}
	tegra_pm_irq_set_wake_type(gpio_to_irq(SIM_CARD_DET),
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	enable_irq_wake(gpio_to_irq(SIM_CARD_DET));

	/* init work queue */
	workqueue = create_singlethread_workqueue
		("ril_workqueue");
	if (!workqueue) {
		pr_err("%s - cannot create workqueue\n", __func__);
		err = -1;
		goto failed3;
	}

	/* init SIM plug functions */
	err = sim_hot_plug_init(dev, workqueue);
	if (err < 0) {
		pr_err("%s - init SIM hotplug failed\n",
			__func__);
		goto failed4;
	}

	/* init modem crash dump functions */
	err = ril_modem_crash_init(dev, workqueue);
	if (err < 0) {
		pr_err("%s - init modem crash handler failed\n",
			__func__);
		goto failed5;
	}

	RIL_INFO("RIL init successfully\n");
	return 0;

failed5:
    sim_hot_plug_exit();
failed4:
	destroy_workqueue(workqueue);
failed3:
	free_irq(gpio_to_irq(SIM_CARD_DET), NULL);
failed2:
	remove_ril_files();
failed1:
	gpio_free_array(ril_gpios_nakasi3g,
			ARRAY_SIZE(ril_gpios_nakasi3g));
	return err;
}

static void __exit ril_exit(void)
{
	RIL_INFO("RIL exit\n");

	if (project_id != GROUPER_PROJECT_NAKASI_3G) {
		RIL_ERR("Ril driver doesn't support this project\n");
		return;
	}

	/* free irq(s) */
	free_irq(gpio_to_irq(SIM_CARD_DET), NULL);

	/* free gpio(s) */
	gpio_free_array(ril_gpios_nakasi3g,
			ARRAY_SIZE(ril_gpios_nakasi3g));

	/* delete device file(s) */
	remove_ril_files();

	/* unregister modem crash handler */
	ril_modem_crash_exit();

	/* unregister SIM hot plug */
	sim_hot_plug_exit();

	/* destroy workqueue */
	destroy_workqueue(workqueue);
}

module_init(ril_init);
module_exit(ril_exit);
