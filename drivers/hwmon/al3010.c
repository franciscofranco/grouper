#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include <linux/ktime.h>

#include <linux/miscdevice.h>

#define AL3010_DRV_NAME	"al3010"
#define DRIVER_VERSION		"1.0"

#define AL3010_NUM_CACHABLE_REGS	9

#define	AL3010_ALS_COMMAND	4	//ref al3010_reg[AL3010_NUM_CACHABLE_REGS]
#define	AL3010_RAN_MASK	0x70
#define	AL3010_RAN_SHIFT	(4)

#define AL3010_MODE_COMMAND	0	//ref al3010_reg[AL3010_NUM_CACHABLE_REGS]
#define AL3010_MODE_SHIFT	(0)
#define AL3010_MODE_MASK	0x07

#define AL3010_POW_MASK		0x01
#define AL3010_POW_UP		0x01
#define AL3010_POW_DOWN		0x00
#define AL3010_POW_SHIFT	(0)

#define	AL3010_ADC_LSB	0x0c
#define	AL3010_ADC_MSB	0x0d


#define CAL_ALS_PATH "/data/lightsensor/AL3010_Config.ini"

bool flagLoadAl3010Config = false;
static int calibration_base_lux = 1000;
static int calibration_regs = 880; // default K value 880 is average K value of PR devices
static int default_calibration_regs = 880;

static bool is_poweron_after_resume = false;
static struct timeval t_poweron_timestamp;

#define AL3010_IOC_MAGIC 0xF3
#define AL3010_IOC_MAXNR 2
#define AL3010_POLL_DATA _IOR(AL3010_IOC_MAGIC,2,int )

#define AL3010_IOCTL_START_HEAVY 2
#define AL3010_IOCTL_START_NORMAL 1
#define AL3010_IOCTL_END 0

#define START_NORMAL	(HZ)
#define START_HEAVY	(HZ)

static int poll_mode=0;
struct delayed_work al3010_poll_data_work;
static struct workqueue_struct *sensor_work_queue;
struct i2c_client *al3010_client;


static u8 al3010_reg[AL3010_NUM_CACHABLE_REGS] = 
	{0x00,0x01,0x0c,0x0d,0x10,0x1a,0x1b,0x1c,0x1d};

static int al3010_range[4] = {77806,19452,4863,1216};

struct al3010_data {
	struct i2c_client *client;
	struct mutex lock;
	struct miscdevice misc_dev;
	struct early_suspend light_sensor_early_suspender;
	u8 reg_cache[AL3010_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
};

static int revise_lux_times = 2;
static bool al3010_hardware_fail = false;

static int al3010_update_calibration();
static int al3010_chip_resume(struct al3010_data *data);
/*
 * register access helpers
 */

static int __al3010_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	return (data->reg_cache[reg] & mask) >> shift;
}

static int __al3010_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;

	if (reg >= AL3010_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	int addr = al3010_reg[reg];
	ret = i2c_smbus_write_byte_data(client, addr, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int al3010_get_range(struct i2c_client *client)
{
	int tmp;
	tmp = __al3010_read_reg(client, AL3010_ALS_COMMAND,
											AL3010_RAN_MASK, AL3010_RAN_SHIFT);;
	return al3010_range[tmp];
}

static int al3010_set_range(struct i2c_client *client, int range)
{
	return __al3010_write_reg(client, AL3010_ALS_COMMAND, 
											AL3010_RAN_MASK, AL3010_RAN_SHIFT, range);
}

/* resolution */
static int al3010_get_resolution(struct i2c_client *client)
{
	return 0;
}

static int al3010_set_resolution(struct i2c_client *client, int res)
{
	return 0;
}

/* mode */
static int al3010_get_mode(struct i2c_client *client)
{
	return __al3010_read_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT);
}

static int al3010_set_mode(struct i2c_client *client, int mode)
{
	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT, mode);
}

/* power_state */
static int al3010_set_power_state(struct i2c_client *client, int state)
{
	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
				AL3010_POW_MASK, AL3010_POW_SHIFT, 
				state ? AL3010_POW_UP : AL3010_POW_DOWN);
}

static int al3010_get_power_state(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	//u8 cmdreg = data->reg_cache[AL3010_MODE_COMMAND];
	// do not use cache data check power state , directly get register data from IC.
	mutex_lock(&data->lock);
	u8 cmdreg = i2c_smbus_read_byte_data(client, AL3010_MODE_COMMAND);
	mutex_unlock(&data->lock);
	return (cmdreg & AL3010_POW_MASK) >> AL3010_POW_SHIFT;
}

static int al3010_get_adc_value(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int lsb, msb, range;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, AL3010_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3010_ADC_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	if(!flagLoadAl3010Config){
		al3010_update_calibration();
		flagLoadAl3010Config = true;
	}

	//range = al3010_get_range(client);
	//printk("light sesnor info : calibration_base_lux = %d\n",calibration_base_lux);
	//printk("light sesnor info : calibration_regs = %d\n",calibration_regs);
	return (u32)( ( ((msb << 8) | lsb) * calibration_base_lux ) /calibration_regs);
	//return (u32)(((msb << 8) | lsb) * range) >> 16;
}

static int al3010_get_reg_value(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int lsb, msb, range;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, AL3010_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3010_ADC_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	range = al3010_get_range(client);
	return (u16)((msb << 8) | lsb);
	//return (u32)(((msb << 8) | lsb) * range) >> 16;
}

/*
 * light sensor calibration
 */

static int al3010_update_calibration()
{
	char buf[256];
	int calibration_value = 0;
	mm_segment_t oldfs;
	oldfs=get_fs();
	set_fs(get_ds());
	memset(buf, 0, sizeof(u8)*256);
	struct file *fp = NULL;
	fp=filp_open(CAL_ALS_PATH, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		int ret = 0;
		ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
		//printk("light sensor info : ret = %d , f_pos = %d",ret ,fp->f_pos);
		//printk("light sensor info : AL3010_Config content is :%s\n", buf);
		sscanf(buf,"%d\n", &calibration_value);
		//printk("light sensor info : calibration_value= %d\n",calibration_value);
		if(calibration_value > 0){
			calibration_regs = calibration_value;
		}
		filp_close(fp, NULL);
		set_fs(oldfs);
		return 0;
	}else{
		return -1;
	}
}

/*
 * sysfs layer
 */

/* power state */
static ssize_t al3010_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	if(al3010_hardware_fail==true){
		return sprintf(buf, "%d\n", 0);
	}
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", al3010_get_power_state(client));
}

/* lux */
static ssize_t al3010_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* No LUX data if not operational */
	if (al3010_get_power_state(client) != 0x01)
		return -EBUSY;

	return sprintf(buf, "%d\n", al3010_get_adc_value(client));
}

/* reg */
static ssize_t al3010_show_reg(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* No LUX data if not operational */
	if (al3010_get_power_state(client) != 0x01)
		return -EBUSY;

	return sprintf(buf, "%d\n", al3010_get_reg_value(client));
}

/* refresh calibration */
static ssize_t al3010_refresh_calibration(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",al3010_update_calibration());
}

/* revise lux */
static ssize_t al3010_show_revise_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	//printk("light sensor al3010 show_revise_lux+\n");
	if(al3010_hardware_fail==true){
		return sprintf(buf, "%d\n", -1);
	}
	struct i2c_client *client = to_i2c_client(dev);

	/* No LUX data if not operational */
	if (al3010_get_power_state(client) != 0x01)
		return -EBUSY;
	//+++ wait al3010 wake up
	if(is_poweron_after_resume == true){
		int require_wait_time = 200;//(ms)
		struct timeval t_current_time;
		int diff_time = 0;
		do_gettimeofday(&t_current_time);
		diff_time = ( (t_current_time.tv_sec-t_poweron_timestamp.tv_sec)*1000000 + (t_current_time.tv_usec-t_poweron_timestamp.tv_usec) )/1000;
		//printk("light sensor debug : first_get_lux_time - later_resume_time = %d ms \n",diff_time);
		int real_wait_time = require_wait_time - diff_time;
		if(real_wait_time>require_wait_time){
			//printk("light sensor debug : first event wait time = %d\n",require_wait_time);
			msleep(require_wait_time);
		}else if(real_wait_time>0){
			//printk("light sensor debug : first event wait time = %d\n",real_wait_time);
                        msleep(real_wait_time);
		}
		is_poweron_after_resume = false;
	}
	//---

	return sprintf(buf, "%d\n", ( al3010_get_adc_value(client)*revise_lux_times ));
}

/* default lux */
static ssize_t al3010_show_default_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

    int show_lux_value = al3010_get_adc_value(client);
    int show_default_lux_value = (show_lux_value*calibration_regs)/default_calibration_regs;
	return sprintf(buf, "%d\n", show_default_lux_value);
}

/* power on*/
static ssize_t al3010_power_on(struct device *dev ,
                                 struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
	struct al3010_data *data = i2c_get_clientdata(client);
        int ret = al3010_chip_resume(data);
	return sprintf(buf, "%d\n", ret);
}

static SENSOR_DEVICE_ATTR(show_reg, 0644, al3010_show_reg, NULL, 1);
static SENSOR_DEVICE_ATTR(show_lux, 0644, al3010_show_lux, NULL, 2);
static SENSOR_DEVICE_ATTR(lightsensor_status, 0644, al3010_show_power_state, NULL, 3);
static SENSOR_DEVICE_ATTR(refresh_cal, 0644, al3010_refresh_calibration, NULL, 4);
static SENSOR_DEVICE_ATTR(show_revise_lux, 0644, al3010_show_revise_lux, NULL, 5);
static SENSOR_DEVICE_ATTR(show_default_lux, 0644, al3010_show_default_lux, NULL, 6);
static SENSOR_DEVICE_ATTR(power_on,0644,al3010_power_on,NULL,7);

static struct attribute *al3010_attributes[] = {
	&sensor_dev_attr_show_reg.dev_attr.attr,
	&sensor_dev_attr_show_lux.dev_attr.attr,
	&sensor_dev_attr_lightsensor_status.dev_attr.attr,
	&sensor_dev_attr_refresh_cal.dev_attr.attr,
	&sensor_dev_attr_show_revise_lux.dev_attr.attr,
	&sensor_dev_attr_show_default_lux.dev_attr.attr,
	&sensor_dev_attr_power_on.dev_attr.attr,
	NULL
};

static const struct attribute_group al3010_attr_group = {
	.attrs = al3010_attributes,
};

static int al3010_chip_suspend(struct al3010_data *data)
{
	int ret = 0;
	ret = al3010_set_power_state(data->client, 0);
	return ret;
}

static int al3010_chip_resume(struct al3010_data *data)
{
	/* restore registers from cache */
	int ret=0;
	if (al3010_get_power_state(data->client) == 0){
	        mutex_lock(&data->lock);
		int i=0;
		for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++)
			if (i2c_smbus_write_byte_data(data->client, i, data->reg_cache[i])){
				mutex_unlock(&data->lock);
				return -EIO;
			}
		mutex_unlock(&data->lock);
		ret = al3010_set_power_state(data->client,1);
                is_poweron_after_resume = true;
                do_gettimeofday(&t_poweron_timestamp);
	}else{
		printk("al3010 debug log : light sensor chip is resumed\n");
	}
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static int al3010_early_suspend(struct early_suspend *h)
{
	if(al3010_hardware_fail==true){
		printk("al3010_early_suspend\n");
		return 0;
	}
	printk("al3010_early_suspend+\n");
	int ret = 0;
	//+++
	struct al3010_data *data = container_of(h, struct al3010_data, light_sensor_early_suspender);
	ret = al3010_chip_suspend(data);
        //---
	printk("al3010_early_suspend-\n");
	return ret;
}

static int al3010_late_resume(struct early_suspend *h)
{
	if(al3010_hardware_fail==true){
		printk("al3010_late_resume\n");
		return 0;
	}
	printk("al3010_late_resume+\n");
	int ret=0;
	//+++
	struct al3010_data *data = container_of(h, struct al3010_data, light_sensor_early_suspender);
    //delay 5ms to avoid al3010_early_suspend and al3010_late_resume too close.
    //if too close , it would cause al3010 chip power on fail
    mdelay(5);
	ret = al3010_chip_resume(data);
	//---
	printk("al3010_late_resume-\n");
	return ret;
}
#endif

static int al3010_init_client(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int err = 0;
	/* set defaults */
	err = al3010_set_power_state(client, 1);
	if(err){
		printk("light sensor err : al3010 set power up err\n");
		return err;
	}
	//set sensor range to 4863 lux.
	//(If panel luminousness is 10% , the range of pad is 0 ~ 48630 lux.)
	err = al3010_set_range(client, 2);
	if(err){
		printk("light sensor err : al3010 set range err\n");
		return err;
	}
	//al3010_set_resolution(client, 0);
	//al3010_set_mode(client, 0);


	return 0;
}

/**
 * i2c stress test
 */
int al3010_open(struct inode *inode, struct file *filp)
{
	printk("light sensor info : %s\n", __func__);
	return 0;
}

int al3010_release(struct inode *inode, struct file *filp)
{
	printk("light sensor info : %s\n", __func__);
	return 0;
}

int al3010_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;

	if (_IOC_TYPE(cmd) != AL3010_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > AL3010_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case AL3010_POLL_DATA:
			if (arg == AL3010_IOCTL_START_HEAVY){
				printk("light sensor info : ioctl heavy\n");
				poll_mode = START_HEAVY;
				queue_delayed_work(sensor_work_queue, &al3010_poll_data_work, poll_mode);
			}
			else if (arg == AL3010_IOCTL_START_NORMAL){
				printk("light sensor info : ioctl normal\n");
				poll_mode = START_NORMAL;
				queue_delayed_work(sensor_work_queue, &al3010_poll_data_work, poll_mode);
			}
			else if  (arg == AL3010_IOCTL_END){
				printk("light sensor info : ioctl end\n");
				cancel_delayed_work_sync(&al3010_poll_data_work);
			}
			else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return 0;
}
struct file_operations al3010_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = al3010_ioctl,
	.open =	al3010_open,
	.release = al3010_release,
};

static void  al3010_poll_data(struct work_struct * work)
{
	int lux = al3010_get_adc_value(al3010_client);
	//printk("FOR TEST , al3010_poll_data light sensor lux = %d\n",lux);
	if(poll_mode ==0)
		msleep(5);

	queue_delayed_work(sensor_work_queue, &al3010_poll_data_work, poll_mode);
}

/*
 * I2C layer
 */

static int __devinit al3010_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct al3010_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct al3010_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	/* initialize the AL3010 chip */
	err = al3010_init_client(client);
	if (err){
		printk("light sensor info : al3010 hardware fail\n");
		printk("light sensor info : keep al3010 driver alive\n");
		err = 0;
		al3010_hardware_fail = true;
		//goto exit_kfree;
	}
	//re-init , workaround to fix init fail when i2c arbitration lost
	if(al3010_hardware_fail == true){
		err = al3010_init_client(client);
		if(err){
			printk("light sensor info : al3010 re-init fail\n");
			printk("light sensor info : keep al3010 driver alive\n");
		}else{
			printk("light sensor info : al3010 re-init success\n");
			al3010_hardware_fail = false;
		}
		err = 0;
	}
	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &al3010_attr_group);
	if (err){
		printk("light sensor err : al3010 init sysfs fail\n");
		goto exit_kfree;
	}

	/* register device node */

	printk("light sensor info : al3010 probe successed\n");
	dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);

	/* init for i2c stress test */
	sensor_work_queue = create_singlethread_workqueue("i2c_lightsensor_wq");
	if(!sensor_work_queue){
		printk("al3010_probe: Unable to create workqueue");
		goto exit_kfree;
	}
	INIT_DELAYED_WORK(&al3010_poll_data_work, al3010_poll_data);
	al3010_client = client;
	data->misc_dev.minor  = MISC_DYNAMIC_MINOR;
	data->misc_dev.name = "lightsensor";
	data->misc_dev.fops = &al3010_fops;
	err = misc_register(&data->misc_dev);
	if (err){
		printk("light sensor err : Unable to register %s\misc device\n",
				data->misc_dev.name);
		goto exit_kfree;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	data->light_sensor_early_suspender.suspend = al3010_early_suspend;
	data->light_sensor_early_suspender.resume = al3010_late_resume;
	data->light_sensor_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB+100;
	register_early_suspend(&data->light_sensor_early_suspender);
#endif
	return 0;

exit_kfree:
	kfree(data);
	return err;
}

static int __devexit al3010_remove(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	misc_deregister(&data->misc_dev);

	sysfs_remove_group(&client->dev.kobj, &al3010_attr_group);
	al3010_set_power_state(client, 0);
	kfree(i2c_get_clientdata(client));
	printk("light sensor info : al3010 remove successed\n");
	return 0;
}

#ifdef CONFIG_PM
static int al3010_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if(al3010_hardware_fail==true){
		printk("al3010_suspend\n");
		return 0;
	}
	printk("al3010_suspend+\n");
	int ret = 0;
	//+++
	struct al3010_data *data = i2c_get_clientdata(client);
	ret = al3010_chip_suspend(data);
	//---
	printk("al3010_suspend-\n");
	return ret;
}

static int al3010_resume(struct i2c_client *client)
{
	if(al3010_hardware_fail==true){
		printk("al3010_resume\n");
		return 0;
	}
	printk("al3010_resume+\n");
	int ret=0;
	//+++
	struct al3010_data *data = i2c_get_clientdata(client);
	ret = al3010_chip_resume(data);
	//---
	printk("al3010_resume-\n");
	return ret;
}

#else
#define al3010_suspend	NULL
#define al3010_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id al3010_id[] = {
	{ "al3010", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3010_id);

static struct i2c_driver al3010_driver = {
	.driver = {
		.name	= AL3010_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = al3010_suspend,
	.resume	= al3010_resume,
	.probe	= al3010_probe,
	.remove	= __devexit_p(al3010_remove),
	.id_table = al3010_id,
};

static int __init al3010_init(void)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	printk("light sensor info : al3010 init \n");
	int ret = i2c_add_driver(&al3010_driver);
	printk(KERN_INFO "%s- #####\n", __func__);
	return ret;
}

static void __exit al3010_exit(void)
{
	printk("light sensor info : al3010 exit \n");
	i2c_del_driver(&al3010_driver);
}

MODULE_AUTHOR("yc");
MODULE_DESCRIPTION("test version v1.0");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3010_init);
module_exit(al3010_exit);

