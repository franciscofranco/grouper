/*
 * An I2C driver for SMSC Proximity Sensor CAP1106.
 *
 * Copyright (c) 2012, ASUSTek Corporation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <asm/gpio.h>

#include "include/mach/board-grouper-misc.h"

MODULE_DESCRIPTION("SMSC Proximity Sensor CAP1106 Driver");
MODULE_LICENSE("GPL");

/*----------------------------------------------------------------------------
 ** Debug Utility
 **----------------------------------------------------------------------------*/
#define PROX_SENSOR_DEBUG 0
#define PROX_SENSOR_VERBOSE_DEBUG 0

#if PROX_SENSOR_DEBUG
#define PROX_DEBUG(format, arg...)	\
    printk(KERN_INFO "CAP1106: [%s] " format , __FUNCTION__ , ## arg)
#else
#define PROX_DEBUG(format, arg...)
#endif

#define PROX_INFO(format, arg...)	\
    printk(KERN_INFO "CAP1106: [%s] " format , __FUNCTION__ , ## arg)
#define PROX_ERROR(format, arg...)	\
    printk(KERN_ERR "CAP1106: [%s] " format , __FUNCTION__ , ## arg)

#define SAR_DET_3G_PR3 139 /* TEGRA_GPIO_PR3 */
#define SAR_DET_3G_PS5      149
#define NAME_RIL_PROX "ril_proximity"

/*----------------------------------------------------------------------------
 ** Global Variable
 **----------------------------------------------------------------------------*/
struct cap1106_data {
    struct attribute_group attrs;
    struct i2c_client *client;
    //struct work_struct work;
    struct delayed_work work;
    struct delayed_work checking_work;
    int enable;
    int obj_detect;
    int overflow_status;
};

static DEFINE_MUTEX(prox_mtx);
static struct cap1106_data *prox_data;
static struct workqueue_struct *prox_wq;
static struct switch_dev prox_sdev;
static long checking_work_period = 100; //default (ms)
static int is_wood_sensitivity = 0;
static int prev_c2_status = 0;
static int prev_c6_status = 0;
static int c2_acc_cnt = 0;
static int c6_acc_cnt = 0;
static int acc_limit = 10;
static int force_enable = 1;

/*----------------------------------------------------------------------------
 ** FUNCTION DECLARATION
 **----------------------------------------------------------------------------*/
static int __devinit cap1106_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit cap1106_remove(struct i2c_client *client);
static int cap1106_suspend(struct i2c_client *client, pm_message_t mesg);
static int cap1106_resume(struct i2c_client *client);
static s32 cap1106_read_reg(struct i2c_client *client, u8 command);
static s32 cap1106_write_reg(struct i2c_client *client, u8 command, u8 value);
static int __init cap1106_init(void);
static void __exit cap1106_exit(void);
static irqreturn_t cap1106_interrupt_handler(int irq, void *dev);
static void cap1106_work_function(struct delayed_work *work);
static int cap1106_init_sensor(struct i2c_client *client);
static int cap1106_config_irq(struct i2c_client *client);
static void cap1106_enable_sensor(struct i2c_client *client, int enable);

/*----------------------------------------------------------------------------
 ** I2C Driver Structure
 **----------------------------------------------------------------------------*/
static const struct i2c_device_id cap1106_id[] = {
    {"cap1106", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, cap1106_id);

static struct i2c_driver cap1106_driver = {
    .driver = {
        .name	= "cap1106",
        .owner	= THIS_MODULE,
    },
    .probe		= cap1106_probe,
    .remove		= __devexit_p(cap1106_remove),
    .resume     = cap1106_resume,
    .suspend    = cap1106_suspend,
    .id_table	= cap1106_id,
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device Attributes Sysfs Show/Store
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_obj_detect(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        ret = sprintf(buf, "%d\n", data->obj_detect);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t show_sensitivity(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x1F);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensitivity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (data->enable) {
        cap1106_write_reg(client, 0x1F, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_gain(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x00);
        ret = sprintf(buf, "%02X\n", (value & 0xC0) >> 6);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_gain(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    long gain_value;
    long reg_value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &gain_value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (data->enable) {
        reg_value = cap1106_read_reg(client, 0x00);
        cap1106_write_reg(client, 0x00, (reg_value & 0x3F) | ((gain_value & 0x3) << 6));
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_2_delta_count(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x11);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t show_sensor_input_2_th(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x31);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_input_2_th(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (data->enable) {
        cap1106_write_reg(client, 0x31, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_6_delta_count(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x15);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t show_sensor_input_6_th(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x35);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_input_6_th(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (data->enable) {
        cap1106_write_reg(client, 0x35, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_noise_th(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x38);
        ret = sprintf(buf, "%02X\n", value & 0x3);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_input_noise_th(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (data->enable) {
        cap1106_write_reg(client, 0x38, value & 0x3);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x03);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t show_sensing_cycle(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x24);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensing_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (data->enable) {
        cap1106_write_reg(client, 0x24, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_onoff(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    ret = sprintf(buf, "%d\n", data->enable);
    mutex_unlock(&prox_mtx);
    return ret;
}

static ssize_t store_sensor_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    long enable;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 10, &enable))
        return -EINVAL;

    if ((enable != 1) && (enable != 0))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    force_enable = enable;
    cap1106_enable_sensor(client, enable);
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_recal(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        value = cap1106_read_reg(client, 0x26);
        ret = sprintf(buf, value == 0x0 ? "OK\n" : "FAIL\n");
    } else {
        ret = sprintf(buf, "FAIL\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_recal(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cap1106_data *data = i2c_get_clientdata(client);
    long value;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (data->enable) {
        cap1106_write_reg(client, 0x26, 0x22);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

DEVICE_ATTR(obj_detect, 0644, show_obj_detect, NULL);
DEVICE_ATTR(sensitivity, 0644, show_sensitivity, store_sensitivity);
DEVICE_ATTR(sensor_gain, 0644, show_sensor_gain, store_sensor_gain);
DEVICE_ATTR(sensor_input_2_delta_count, 0644, show_sensor_input_2_delta_count, NULL);
DEVICE_ATTR(sensor_input_2_th, 0644, show_sensor_input_2_th, store_sensor_input_2_th);
DEVICE_ATTR(sensor_input_6_delta_count, 0644, show_sensor_input_6_delta_count, NULL);
DEVICE_ATTR(sensor_input_6_th, 0644, show_sensor_input_6_th, store_sensor_input_6_th);
DEVICE_ATTR(sensor_input_noise_th, 0644, show_sensor_input_noise_th, store_sensor_input_noise_th);
DEVICE_ATTR(sensor_input_status, 0644, show_sensor_input_status, NULL);
DEVICE_ATTR(sensing_cycle, 0644, show_sensing_cycle, store_sensing_cycle);
DEVICE_ATTR(sensor_onoff, 0644, show_sensor_onoff, store_sensor_onoff);
DEVICE_ATTR(sensor_recal, 0644, show_sensor_recal, store_sensor_recal);

static struct attribute *cap1106_attr[] = {
    &dev_attr_obj_detect.attr,
    &dev_attr_sensitivity.attr,
    &dev_attr_sensor_gain.attr,
    &dev_attr_sensor_input_2_delta_count.attr,
    &dev_attr_sensor_input_2_th.attr,
    &dev_attr_sensor_input_6_delta_count.attr,
    &dev_attr_sensor_input_6_th.attr,
    &dev_attr_sensor_input_noise_th.attr,
    &dev_attr_sensor_input_status.attr,
    &dev_attr_sensing_cycle.attr,
    &dev_attr_sensor_onoff.attr,
    &dev_attr_sensor_recal.attr,
    NULL
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callbacks for switch device
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t print_prox_name(struct switch_dev *sdev, char *buf)
{
    return sprintf(buf, "%s\n", "prox_sar_det");
}

static ssize_t print_prox_state(struct switch_dev *sdev, char *buf)
{
    int state = -1;
    if (switch_get_state(sdev))
        state = 1;
    else
        state = 0;

    return sprintf(buf, "%d\n", state);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if PROX_SENSOR_VERBOSE_DEBUG
static void dump_registers(struct i2c_client *client)
{
    int value;
    value = cap1106_read_reg(client, 0x00);
    PROX_ERROR("=== Main Control(0x00) is %x\n", value);
    value = cap1106_read_reg(client, 0x02);
    PROX_ERROR("=== Genaral Status(0x02) is %x\n", value);
    value = cap1106_read_reg(client, 0x03);
    PROX_ERROR("=== Sensor Input Status(0x03) is %x\n", value);
    value = cap1106_read_reg(client, 0x0A);
    PROX_ERROR("=== Noise Flag Status(0x0A) is %x\n", value);
    value = cap1106_read_reg(client, 0x21);
    PROX_ERROR("=== Sensor Input Enable Register(0x21) is %x\n", value);
    value = cap1106_read_reg(client, 0x44);
    PROX_ERROR("=== configuration 2(0x44) is %x\n", value);
    value = cap1106_read_reg(client, 0xFD);
    PROX_ERROR("=== Product ID(0xFD) is %x\n", value);
    value = cap1106_read_reg(client, 0xFE);
    PROX_ERROR("=== Manufacturer ID(0xFE) is %x\n", value);
    value = cap1106_read_reg(client, 0xFF);
    PROX_ERROR("=== Revision (0xFF) is %x\n", value);
}
#endif

static void cap1106_enable_sensor(struct i2c_client *client, int enable)
{
    long reg_value;
    //long status;

    struct cap1106_data *data = i2c_get_clientdata(client);

    if (data->enable != enable) {
        reg_value = cap1106_read_reg(client, 0x00);
        if (enable) {
            cap1106_write_reg(client, 0x00, (reg_value & 0xEF) | (!enable << 4));
            // Time to first conversion is 200ms (Max)
            queue_delayed_work(prox_wq, &data->work, msecs_to_jiffies(200));
            enable_irq(client->irq);
            queue_delayed_work(prox_wq, &prox_data->checking_work, checking_work_period);
        } else {
            disable_irq(client->irq);
            cancel_delayed_work_sync(&data->work);
            cancel_delayed_work_sync(&data->checking_work);
            flush_workqueue(prox_wq);
            switch_set_state(&prox_sdev, 0);
            cap1106_write_reg(client, 0x00, (reg_value & 0xEF) | (!enable << 4));
        }
        data->enable = enable;
    }
}

static s32 cap1106_read_reg(struct i2c_client *client, u8 command)
{
    return i2c_smbus_read_byte_data(client, command);
}

static s32 cap1106_write_reg(struct i2c_client *client, u8 command, u8 value)
{
    return i2c_smbus_write_byte_data(client, command, value);
}

static void cap1106_work_function(struct delayed_work *work)
{
    int status;
    int value_delta_2,value_delta_6;
    int bc2, bc6;
    struct cap1106_data *data = container_of(work, struct cap1106_data, work);

    disable_irq(data->client->irq);
    cap1106_write_reg(data->client, 0x00, 0x80); // Clear INT and Set Gain to MAX
    status = cap1106_read_reg(data->client, 0x03);
    value_delta_2 = cap1106_read_reg(prox_data->client, 0x11);
    value_delta_6 = cap1106_read_reg(prox_data->client, 0x15);
    bc2 = cap1106_read_reg(prox_data->client, 0x51);
    bc6 = cap1106_read_reg(prox_data->client, 0x55);
    PROX_DEBUG("Status: 0x%02X, BC2=0x%02X, D2=0x%02X, BC6=0x%02X, D6=0x%02X\n", status, bc2, value_delta_2, bc6, value_delta_6);
    if (is_wood_sensitivity == 0) {
        data->obj_detect = ((status == 0x2) || (status == 0x20) || (status == 0x22));
        switch_set_state(&prox_sdev, data->obj_detect);
        if ((status == 0x2 && value_delta_2 == 0x7F)
            || (status == 0x20 && value_delta_6 == 0x7F)
            || (status == 0x22 && (value_delta_2 == 0x7F || value_delta_6 == 0x7F))) {
            PROX_DEBUG("set to wood sensitivity------>\n");
            //set sensitivity and threshold for wood touch
            cap1106_write_reg(prox_data->client, 0x1f, 0x4f);
            cap1106_write_reg(prox_data->client, 0x31, 0x50);
            cap1106_write_reg(prox_data->client, 0x35, 0x50);
            is_wood_sensitivity = 1;
            data->overflow_status = status;
            c2_acc_cnt = 0;
            c6_acc_cnt = 0;
        } else {
            if (value_delta_2 >= 0x08 && value_delta_2 <= 0x3F)
                c2_acc_cnt++;
            if (value_delta_6 >= 0x0a && value_delta_6 <= 0x3F)
                c6_acc_cnt++;

            PROX_DEBUG("c2_acc_cnt=%d, c6_acc_cnt=%d\n", c2_acc_cnt, c6_acc_cnt);
            if (c2_acc_cnt >= acc_limit || c6_acc_cnt >= acc_limit) {
                PROX_DEBUG("+++ FORCE RECALIBRATION +++\n");
                cap1106_write_reg(data->client, 0x26, 0x22);
                c2_acc_cnt = 0;
                c6_acc_cnt = 0;
            }
        }
        prev_c2_status = (status & 0x02);
        prev_c6_status = (status & 0x20);
    }
    enable_irq(data->client->irq);
}

static irqreturn_t cap1106_interrupt_handler(int irq, void *dev)
{
    struct cap1106_data *data = i2c_get_clientdata(dev);

    //PROX_DEBUG("\n");

    queue_delayed_work(prox_wq, &data->work, 0);
    return IRQ_HANDLED;
}

static int cap1106_config_irq(struct i2c_client *client)
{
    int rc = 0 ;
    unsigned gpio = irq_to_gpio(client->irq);
    const char* label = "cap1106_alert";

    PROX_DEBUG("\n");

    rc = gpio_request(gpio, "cap1106_alert");
    if (rc) {
        PROX_ERROR("%s: gpio_request failed for gpio %s\n", __func__, "SAR_DET_3G_PR3_GPIO");
        goto err_gpio_request_failed;
    }

    rc = gpio_direction_input(gpio) ;
    if (rc) {
        PROX_ERROR("%s: gpio_direction_input failed for gpio %s\n", __func__, "SAR_DET_3G_PR3_GPIO");
        goto err_gpio_direction_input_failed;
    }

    rc = request_irq(client->irq, cap1106_interrupt_handler, IRQF_TRIGGER_FALLING, label, client);
    if(rc){
        PROX_ERROR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, client->irq, rc);
        goto err_gpio_request_irq_failed;
    }

    PROX_INFO("INT configuration done, GPIO=%d, IRQ=%d\n", gpio, client->irq);

#if PROX_SENSOR_VERBOSE_DEBUG
    dump_registers(client);
#endif

    return 0 ;
err_gpio_request_irq_failed:
err_gpio_direction_input_failed:
    gpio_free(gpio);
err_gpio_request_failed:
    return rc;
}

static int cap1106_init_sensor(struct i2c_client *client)
{
    u8 bIdx;
    int rc = 0;
    const u8 InitTable[] = {
        0x1f, 0x1f, // Data sensitivity (need to be fine tune for real system).
        0x20, 0x20, // MAX duration disable
        0x21, 0x22, // Enable CS2+CS6.
        0x22, 0xff, // MAX duration time to max , repeat period time to max
        0x24, 0x39, // digital count update time to 140*64ms
        0x27, 0x22, // Enable INT. for CS2+CS6.
        0x28, 0x22, // disable repeat irq
        0x2a, 0x00, // all channel run in the same time
        0x31, 0x08, // Threshold of CS 2 (need to be fine tune for real system).
        0x35, 0x0a, // Threshold of CS 6 (need to be fine tune for real system).
        0x26, 0x22, // force re-cal
        0x00, 0x00, // Reset INT. bit.
    };

    PROX_DEBUG("client->name: %s, client->addr: 0x%X, irq: 0x%X\n", client->name, client->addr, client->irq);

    for (bIdx = 0; bIdx < sizeof(InitTable) / sizeof(InitTable[0]); bIdx += 2) {
        if ((rc = cap1106_write_reg(client, InitTable[bIdx],
                        InitTable[bIdx + 1]))) {
            PROX_ERROR("=== Write Error, rc=0x%X\n", rc);
            break;
        }
    }

    PROX_INFO("SAR_DET_3G_PR3 = %d(0: ON)\n", gpio_get_value(SAR_DET_3G_PR3));

#if PROX_SENSOR_VERBOSE_DEBUG
    dump_registers(client);
#endif

    return rc;
}

static void cap1106_checking_work_function(struct delayed_work *work) {
    int status;
    int value_delta_2;
    int value_delta_6;
    int bc2, bc6;

    if (is_wood_sensitivity == 1){
        mutex_lock(&prox_mtx);
        if (prox_data->enable) {
            status = cap1106_read_reg(prox_data->client, 0x03);
            value_delta_2 = cap1106_read_reg(prox_data->client, 0x11);
            value_delta_6 = cap1106_read_reg(prox_data->client, 0x15);
            bc2 = cap1106_read_reg(prox_data->client, 0x51);
            bc6 = cap1106_read_reg(prox_data->client, 0x55);
            PROX_DEBUG("Status: 0x%02X, BC2=0x%02X, D2=0x%02X, BC6=0x%02X, D6=0x%02X\n", status, bc2, value_delta_2, bc6, value_delta_6);
            if ((value_delta_2 == 0x00 && value_delta_6 == 0x00)
                || (value_delta_2 == 0xFF && value_delta_6 == 0xFF)
                || (value_delta_2 == 0x00 && value_delta_6 == 0xFF)
                || (value_delta_2 == 0xFF && value_delta_6 == 0x00)
                || (prox_data->overflow_status == 0x2 && (value_delta_2 > 0x50) && (value_delta_2 <= 0x7F))
                || (prox_data->overflow_status == 0x20 && (value_delta_6 > 0x50) && (value_delta_6 <= 0x7F))
                || (prox_data->overflow_status == 0x22 && (((value_delta_2 > 0x50) && (value_delta_2 <= 0x7F))
                                                            || ((value_delta_6 > 0x50) && (value_delta_6 <= 0x7F))))) {
                PROX_DEBUG("unset is_wood_sensitivity to 0\n");
                //set sensitivity and threshold for 2cm body distance
                cap1106_write_reg(prox_data->client, 0x1f, 0x1f);
                cap1106_write_reg(prox_data->client, 0x31, 0x08);
                cap1106_write_reg(prox_data->client, 0x35, 0x0a);
                is_wood_sensitivity = 0;
                queue_delayed_work(prox_wq, &prox_data->work, 0);
            }
        } else {
            PROX_DEBUG("delta 2 = -1\n");
        }
        mutex_unlock(&prox_mtx);
    }
    queue_delayed_work(prox_wq, &prox_data->checking_work, checking_work_period);
}

static int __devinit cap1106_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;

    PROX_DEBUG("\n");

    prox_data->client = client;

    /* Touch data processing workqueue initialization */
    INIT_DELAYED_WORK(&prox_data->work, cap1106_work_function);
    INIT_DELAYED_WORK(&prox_data->checking_work,cap1106_checking_work_function);

    i2c_set_clientdata(client, prox_data);
    prox_data->client->flags = 0;
    strlcpy(prox_data->client->name, "cap1106", I2C_NAME_SIZE);
    prox_data->enable = 0;

    rc = cap1106_init_sensor(prox_data->client);
    if (rc) {
        PROX_ERROR("Sensor initialization failed!\n");
        goto err_init_sensor_failed;
    }

    prox_data->attrs.attrs = cap1106_attr;
    rc = sysfs_create_group(&prox_data->client->dev.kobj, &prox_data->attrs);
    if (rc) {
        PROX_ERROR("Create the sysfs group failed!\n");
        goto err_create_sysfs_group_failed;
    }

    /* register switch class */
    prox_sdev.name = NAME_RIL_PROX;
    prox_sdev.print_name = print_prox_name;
    prox_sdev.print_state = print_prox_state;

    rc = switch_dev_register(&prox_sdev);

    if (rc) {
        PROX_ERROR("Switch device registration failed!\n");
        goto err_register_switch_class_failed;
    }

    rc = cap1106_config_irq(prox_data->client);
    if (rc) {
        PROX_ERROR("Sensor INT configuration failed!\n");
        goto err_config_irq_failed;
    }

    prox_data->enable = 1;
    prox_data->overflow_status = 0x0;
    queue_delayed_work(prox_wq, &prox_data->work, msecs_to_jiffies(200));
    queue_delayed_work(prox_wq, &prox_data->checking_work, checking_work_period);

    return 0;

err_config_irq_failed:
err_register_switch_class_failed:
    sysfs_remove_group(&prox_data->client->dev.kobj, &prox_data->attrs);
err_create_sysfs_group_failed:
err_init_sensor_failed:
    return rc;
}

static int __devexit cap1106_remove(struct i2c_client *client)
{
    PROX_DEBUG("\n");
    switch_dev_unregister(&prox_sdev);
    sysfs_remove_group(&client->dev.kobj, &prox_data->attrs);
    free_irq(client->irq, client);
    kfree(prox_data);
    return 0;
}

static int cap1106_suspend(struct i2c_client *client, pm_message_t mesg)
{
    PROX_DEBUG("+\n");
    mutex_lock(&prox_mtx);
    cap1106_enable_sensor(client, 0);
    mutex_unlock(&prox_mtx);
    PROX_DEBUG("-\n");
    return 0;
}

static int cap1106_resume(struct i2c_client *client)
{
    PROX_DEBUG("+\n");
    mutex_lock(&prox_mtx);
    if (force_enable)
        cap1106_enable_sensor(client, 1);
    mutex_unlock(&prox_mtx);
    PROX_DEBUG("-\n");
    return 0;
}

static int __init cap1106_init(void)
{
    int rc;

    PROX_DEBUG("\n");

    if (GROUPER_PROJECT_BACH != grouper_get_project_id()) {
        PROX_ERROR("Cap1106 driver doesn't support this project\n");
        return -1;
    }

    prox_wq = create_singlethread_workqueue("prox_wq");
    if(!prox_wq) {
        PROX_ERROR("create_singlethread_workqueue failed!\n");
        rc = -ENOMEM;
        goto err_create_singlethread_workqueue_failed;
    }

    prox_data = kzalloc(sizeof(struct cap1106_data), GFP_KERNEL);
    if (!prox_data) {
        PROX_ERROR("kzalloc failed!\n");
        rc = -ENOMEM;
        goto err_kzalloc_failed;
    }

    rc = i2c_add_driver(&cap1106_driver);
    if (rc) {
        PROX_ERROR("i2c_add_driver failed!\n");
        goto err_i2c_add_driver_failed;
    }

    tegra_gpio_enable(SAR_DET_3G_PR3);
    tegra_gpio_disable(SAR_DET_3G_PS5);

    PROX_INFO("Driver intialization done, SAR_DET_3G_PR3=%d", gpio_get_value(SAR_DET_3G_PR3));

    return 0;

err_i2c_add_driver_failed:
    kfree(prox_data);
err_kzalloc_failed:
    destroy_workqueue(prox_wq);
err_create_singlethread_workqueue_failed:
    return rc;
}

static void __exit cap1106_exit(void)
{
    PROX_DEBUG("\n");

    if (GROUPER_PROJECT_BACH != grouper_get_project_id()) {
        PROX_ERROR("Cap1106 driver doesn't support this project\n");
        return;
    }

    i2c_del_driver(&cap1106_driver);

    if (prox_wq)
        destroy_workqueue(prox_wq);
}

module_init(cap1106_init);
module_exit(cap1106_exit);
