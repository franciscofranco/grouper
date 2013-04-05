/*
 * Copyright 2013 Francisco Franco
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#define TEGRA_HOTPLUG_CONTROL_VERSION 1

unsigned int first_level = 90;
unsigned int second_level = 50;
unsigned int third_level = 30;

extern void update_first_level(unsigned int level);
extern void update_second_level(unsigned int level);
extern void update_third_level(unsigned int level);

/*
 * Sysfs get/set entries
 */

static ssize_t first_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", first_level);
}

static ssize_t first_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int new_val;
    
	sscanf(buf, "%u", &new_val);
    
    if (new_val != first_level && new_val >= 0 && new_val <= 100)
    {
        update_first_level(new_val);
        first_level = new_val;
    }
    
    return size;
}

static ssize_t second_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", second_level);
}

static ssize_t second_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int new_val;
    
	sscanf(buf, "%u", &new_val);
    
    if (new_val != second_level && new_val >= 0 && new_val <= 100)
    {
        update_second_level(new_val);
        second_level = new_val;
    }
    
    return size;
}

static ssize_t third_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", third_level);
}

static ssize_t third_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int new_val;
    
	sscanf(buf, "%u", &new_val);
    
    if (new_val != third_level && new_val >= 0 && new_val <= 100)
    {
        update_third_level(new_val);
        third_level = new_val;
    }
    
    return size;
}

static ssize_t tegra_hotplug_control_version(struct device *dev, struct device_attribute* attr, char *buf)
{
    return sprintf(buf, "%d\n", TEGRA_HOTPLUG_CONTROL_VERSION);
}

static DEVICE_ATTR(first_level, 0777, first_level_show, first_level_store);
static DEVICE_ATTR(second_level, 0777, second_level_show, second_level_store);
static DEVICE_ATTR(third_level, 0777, third_level_show, third_level_store);

static DEVICE_ATTR(version, 0777 , tegra_hotplug_control_version, NULL);

static struct attribute *tegra_hotplug_control_attributes[] =
{
	&dev_attr_first_level.attr,
    &dev_attr_second_level.attr,
    &dev_attr_third_level.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group tegra_hotplug_control_group =
{
	.attrs  = tegra_hotplug_control_attributes,
};

static struct miscdevice tegra_hotplug_control_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tegra_hotplug_control",
};

static int __init tegra_hotplug_control_init(void)
{
    int ret;
    
    pr_info("%s misc_register(%s)\n", __FUNCTION__, tegra_hotplug_control_device.name);
    
    ret = misc_register(&tegra_hotplug_control_device);
    
    if (ret)
    {
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, tegra_hotplug_control_device.name);
	    return 1;
	}
    
    if (sysfs_create_group(&tegra_hotplug_control_device.this_device->kobj, &tegra_hotplug_control_group) < 0)
    {
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", tegra_hotplug_control_device.name);
	}
    
    return 0;
}
late_initcall(tegra_hotplug_control_init);
