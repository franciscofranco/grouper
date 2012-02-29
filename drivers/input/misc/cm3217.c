/* drivers/input/misc/cm3217.c - cm3217 optical sensors driver
 *
 * Copyright (C) 2011 Capella Microsystems Inc.
 * Author: Frank Hsieh <pengyueh@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/cm3217.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#define D(x...)			pr_info(x)

#define I2C_RETRY_COUNT		10

#define LS_POLLING_DELAY	500

static void report_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_work, report_do_work);

struct cm3217_info {
	struct class *cm3217_class;
	struct device *ls_dev;
	struct input_dev *ls_input_dev;

	struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int als_enable;
	int als_enabled_before_suspend;
	uint16_t *adc_table;
	uint16_t cali_table[10];
	int irq;
	int ls_calibrate;
	int (*power) (int, uint8_t);	/* power to the chip */

	uint32_t als_kadc;
	uint32_t als_gadc;
	uint16_t golden_adc;

	int lightsensor_opened;
	int current_level;
	uint16_t current_adc;
	int polling_delay;
};

struct cm3217_info *lp_info;

int enable_log;
int fLevel = -1;

static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;

static int lightsensor_enable(struct cm3217_info *lpi);
static int lightsensor_disable(struct cm3217_info *lpi);

int32_t als_kadc;

static int I2C_RxData(uint16_t slaveAddr, uint8_t *rxData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msgs[] = {
		{
			.addr = slaveAddr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 1) > 0)
			break;
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ERR][CM3217 error] %s retry over %d\n",
		       __func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msg[] = {
		{
			.addr = slaveAddr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ERR][CM3217 error] %s retry over %d\n",
		       __func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm3217_I2C_Read_Byte(uint16_t slaveAddr, uint8_t *pdata)
{
	uint8_t buffer = 0;
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, &buffer, 1);
	if (ret < 0) {
		pr_err("[ERR][CM3217 error]%s: I2C_RxData fail, slave addr: 0x%x\n",
		       __func__, slaveAddr);
		return ret;
	}

	*pdata = buffer;

#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM3217] %s: I2C_RxData[0x%x] = 0x%x\n",
	       __func__, slaveAddr, buffer);
#endif

	return ret;
}

static int _cm3217_I2C_Write_Byte(uint16_t SlaveAddress, uint8_t data)
{
	char buffer[2];
	int ret = 0;

#if 0
	/* Debug use */
	printk(KERN_DEBUG
	       "[CM3217] %s: _cm3217_I2C_Write_Byte[0x%x, 0x%x, 0x%x]\n",
	       __func__, SlaveAddress, cmd, data);
#endif

	buffer[0] = data;

	ret = I2C_TxData(SlaveAddress, buffer, 1);
	if (ret < 0) {
		pr_err("[ERR][CM3217 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	uint8_t lsb, msb;
	int ret = 0;
	struct cm3217_info *lpi = lp_info;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: LSB */
	ret = _cm3217_I2C_Read_Byte(ALS_R_LSB_addr, &lsb);
	if (ret < 0) {
		pr_err("[LS][CM3217 error]%s: _cm3217_I2C_Read_Byte LSB fail\n",
		       __func__);
		return -EIO;
	}

	/* Read ALS data: MSB */
	ret = _cm3217_I2C_Read_Byte(ALS_R_MSB_addr, &msb);
	if (ret < 0) {
		pr_err("[LS][CM3217 error]%s: _cm3217_I2C_Read_Byte MSB fail\n",
		       __func__);
		return -EIO;
	}

	*als_step = (uint16_t) msb;
	*als_step <<= 8;
	*als_step |= (uint16_t) lsb;

	D("[LS][CM3217] %s: raw adc = 0x%X\n", __func__, *als_step);

	if (!lpi->ls_calibrate) {
		*als_step = (*als_step) * lpi->als_gadc / lpi->als_kadc;
		if (*als_step > 0xFFFF)
			*als_step = 0xFFFF;
	}

	return ret;
}

static void report_lsensor_input_event(struct cm3217_info *lpi, bool resume)
{
	uint16_t adc_value = 0;
	int level = 0, i, ret = 0;

	mutex_lock(&als_get_adc_mutex);

	ret = get_ls_adc_value(&adc_value, resume);

	if (lpi->ls_calibrate) {
		for (i = 0; i < 10; i++) {
			if (adc_value <= (*(lpi->cali_table + i))) {
				level = i;
				if (*(lpi->cali_table + i))
					break;
			}
			/* avoid i = 10, because 'cali_table' of size is 10 */
			if (i == 9) {
				level = i;
				break;
			}
		}
	} else {
		for (i = 0; i < 10; i++) {
			if (adc_value <= (*(lpi->adc_table + i))) {
				level = i;
				if (*(lpi->adc_table + i))
					break;
			}
			/* avoid i = 10, because 'cali_table' of size is 10 */
			if (i == 9) {
				level = i;
				break;
			}
		}
	}

	if ((i == 0) || (adc_value == 0))
		D("[LS][CM3217] %s: ADC=0x%03X, Level=%d, l_thd equal 0, "
		  "h_thd = 0x%x\n", __func__, adc_value, level,
		  *(lpi->cali_table + i));
	else
		D("[LS][CM3217] %s: ADC=0x%03X, Level=%d, l_thd = 0x%x, "
		  "h_thd = 0x%x\n", __func__, adc_value, level,
		  *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));

	lpi->current_level = level;
	lpi->current_adc = adc_value;

	/*
	D("[CM3217] %s: *(lpi->cali_table + (i - 1)) + 1 = 0x%X, "
	  "*(lpi->cali_table + i) = 0x%x\n", __func__,
	  *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));
	*/

	if (fLevel >= 0) {
		D("[LS][CM3217] L-sensor force level enable level=%d "
		  "fLevel=%d\n", level, fLevel);
		level = fLevel;
	}

	input_report_abs(lpi->ls_input_dev, ABS_MISC, level);
	input_sync(lpi->ls_input_dev);

	mutex_unlock(&als_get_adc_mutex);
}

static void report_do_work(struct work_struct *work)
{
	struct cm3217_info *lpi = lp_info;

	if (enable_log)
		D("[CM3217] %s\n", __func__);

	report_lsensor_input_event(lpi, 0);

	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
}

static int als_power(int enable)
{
	struct cm3217_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

void lightsensor_set_kvalue(struct cm3217_info *lpi)
{
	if (!lpi) {
		pr_err("[LS][CM3217 error]%s: ls_info is empty\n", __func__);
		return;
	}

	D("[LS][CM3217] %s: ALS calibrated als_kadc=0x%x\n",
	  __func__, als_kadc);

	if (als_kadc >> 16 == ALS_CALIBRATED)
		lpi->als_kadc = als_kadc & 0xFFFF;
	else {
		lpi->als_kadc = 0;
		D("[LS][CM3217] %s: no ALS calibrated\n", __func__);
	}

	if (lpi->als_kadc && lpi->golden_adc > 0) {
		lpi->als_kadc = (lpi->als_kadc > 0 && lpi->als_kadc < 0x1000) ?
				lpi->als_kadc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	} else {
		lpi->als_kadc = 1;
		lpi->als_gadc = 1;
	}

	D("[LS][CM3217] %s: als_kadc=0x%x, als_gadc=0x%x\n",
	  __func__, lpi->als_kadc, lpi->als_gadc);
}

static int lightsensor_update_table(struct cm3217_info *lpi)
{
	uint32_t tmpData[10];
	int i;

	for (i = 0; i < 10; i++) {
		tmpData[i] = (uint32_t) (*(lpi->adc_table + i))
			     * lpi->als_kadc / lpi->als_gadc;
		if (tmpData[i] <= 0xFFFF)
			lpi->cali_table[i] = (uint16_t) tmpData[i];
		else
			lpi->cali_table[i] = 0xFFFF;
		D("[LS][CM3217] %s: Calibrated adc_table: data[%d], %x\n",
		  __func__, i, lpi->cali_table[i]);
	}

	return 0;
}

static int lightsensor_enable(struct cm3217_info *lpi)
{
	int ret = 0;
	uint8_t cmd = 0;

	mutex_lock(&als_enable_mutex);

	D("[LS][CM3217] %s\n", __func__);

	cmd = (CM3217_ALS_IT_2_T | CM3217_ALS_BIT5_Default_1 |
	       CM3217_ALS_WDM_DEFAULT_1);
	ret = _cm3217_I2C_Write_Byte(ALS_W_CMD1_addr, cmd);
	if (ret < 0)
		pr_err("[LS][CM3217 error]%s: set auto light sensor fail\n",
		       __func__);
	else {
		msleep(50);	/* wait for 50 ms for the first report adc */

		/* report an invalid value first to ensure we
		 * trigger an event when adc_level is zero.
		 */
		input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
		input_sync(lpi->ls_input_dev);
		/* resume, IOCTL and DEVICE_ATTR */
		report_lsensor_input_event(lpi, 1);
		lpi->als_enable = 1;
	}

	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
	lpi->als_enable = 1;

	mutex_unlock(&als_enable_mutex);

	return ret;
}

static int lightsensor_disable(struct cm3217_info *lpi)
{
	int ret = 0;
	char cmd = 0;

	mutex_lock(&als_disable_mutex);

	D("[LS][CM3217] %s\n", __func__);

	cmd = (CM3217_ALS_IT_2_T | CM3217_ALS_BIT5_Default_1 |
	       CM3217_ALS_WDM_DEFAULT_1 | CM3217_ALS_SD);
	ret = _cm3217_I2C_Write_Byte(ALS_W_CMD1_addr, cmd);
	if (ret < 0)
		pr_err("[LS][CM3217 error]%s: disable auto light sensor fail\n",
		       __func__);
	else {
		lpi->als_enable = 0;
	}

	cancel_delayed_work(&report_work);
	lpi->als_enable = 0;

	mutex_unlock(&als_disable_mutex);

	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm3217_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM3217] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM3217 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;

	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm3217_info *lpi = lp_info;

	D("[LS][CM3217] %s\n", __func__);
	lpi->lightsensor_opened = 0;

	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	int rc, val;
	struct cm3217_info *lpi = lp_info;

	/* D("[CM3217] %s cmd %d\n", __func__, _IOC_NR(cmd)); */

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		D("[LS][CM3217] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
		  __func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;

	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		D("[LS][CM3217] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
		  __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;

	default:
		pr_err("[LS][CM3217 error]%s: invalid cmd %d\n",
		       __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_adc_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm3217_info *lpi = lp_info;

	D("[LS][CM3217] %s: ADC = 0x%04X, Level = %d\n",
	  __func__, lpi->current_adc, lpi->current_level);

	ret = sprintf(buf, "ADC[0x%04X] => level %d\n",
		      lpi->current_adc, lpi->current_level);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3217_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n", lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm3217_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;

	if (ls_auto)
		ret = lightsensor_enable(lpi);
	else
		ret = lightsensor_disable(lpi);

	D("[LS][CM3217] %s: lpi->als_enable = %d, lpi->ls_calibrate = %d, "
	  "ls_auto=%d\n", __func__, lpi->als_enable, lpi->ls_calibrate,
	  ls_auto);

	if (ret < 0)
		pr_err("[LS][CM3217 error]%s: set auto light sensor fail\n",
		       __func__);

	return count;
}

static DEVICE_ATTR(ls_auto, 0664, ls_enable_show, ls_enable_store);

static ssize_t ls_kadc_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct cm3217_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x", lpi->als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cm3217_info *lpi = lp_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);

	/* if (kadc_temp <= 0 || lpi->golden_adc <= 0) {
		printk(KERN_ERR "[LS][CM3217 error] %s: kadc_temp=0x%x, "
		       "als_gadc=0x%x\n", __func__, kadc_temp,
		       lpi->golden_adc);
		return -EINVAL;
	} */

	mutex_lock(&als_get_adc_mutex);

	if (kadc_temp != 0) {
		lpi->als_kadc = kadc_temp;
		if (lpi->als_gadc != 0) {
			if (lightsensor_update_table(lpi) < 0)
				printk(KERN_ERR
				       "[LS][CM3217 error] %s: "
				       "update ls table fail\n", __func__);
		} else {
			printk(KERN_INFO
			       "[LS]%s: als_gadc =0x%x wait to be set\n",
			       __func__, lpi->als_gadc);
		}
	} else {
		printk(KERN_INFO "[LS]%s: als_kadc can't be set to zero\n",
		       __func__);
	}

	mutex_unlock(&als_get_adc_mutex);

	return count;
}

static DEVICE_ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store);

static ssize_t ls_gadc_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct cm3217_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "gadc = 0x%x\n", lpi->als_gadc);

	return ret;
}

static ssize_t ls_gadc_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cm3217_info *lpi = lp_info;
	int gadc_temp = 0;

	sscanf(buf, "%d", &gadc_temp);

	/* if (gadc_temp <= 0 || lpi->golden_adc <= 0) {
		printk(KERN_ERR "[LS][CM3217 error] %s: kadc_temp=0x%x, "
		       "als_gadc=0x%x\n", __func__, kadc_temp,
		       lpi->golden_adc);
		return -EINVAL;
	} */

	mutex_lock(&als_get_adc_mutex);

	if (gadc_temp != 0) {
		lpi->als_gadc = gadc_temp;
		if (lpi->als_kadc != 0) {
			if (lightsensor_update_table(lpi) < 0)
				printk(KERN_ERR
				       "[LS][CM3217 error] %s: "
				       "update ls table fail\n", __func__);
		} else {
			printk(KERN_INFO
			       "[LS]%s: als_kadc =0x%x wait to be set\n",
			       __func__, lpi->als_kadc);
		}
	} else {
		printk(KERN_INFO "[LS]%s: als_gadc can't be set to zero\n",
		       __func__);
	}

	mutex_unlock(&als_get_adc_mutex);

	return count;
}

static DEVICE_ATTR(ls_gadc, 0664, ls_gadc_show, ls_gadc_store);

static ssize_t ls_adc_table_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	int i;

	for (i = 0; i < 10; i++) {
		length += sprintf(buf + length,
				  "[CM3217]Get adc_table[%d] =  0x%x ; %d, "
				  "Get cali_table[%d] =  0x%x ; %d,\n",
				  i, *(lp_info->adc_table + i),
				  *(lp_info->adc_table + i),
				  i, *(lp_info->cali_table + i),
				  *(lp_info->cali_table + i));
	}

	return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cm3217_info *lpi = lp_info;
	char *token[10];
	unsigned long tempdata[10];
	int i, r;

	printk(KERN_INFO "[LS][CM3217]%s\n", buf);
	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		r = kstrtoul(token[i], 16, &tempdata[i]);
		if (tempdata[i] < 1 || tempdata[i] > 0xffff || r) {
			printk(KERN_ERR
			       "[LS][CM3217 error] adc_table[%d] = "
			       "0x%lx Err\n", i, tempdata[i]);
			return count;
		}
	}

	mutex_lock(&als_get_adc_mutex);

	for (i = 0; i < 10; i++) {
		lpi->adc_table[i] = tempdata[i];
		printk(KERN_INFO
		       "[LS][CM3217]Set lpi->adc_table[%d] =  0x%x\n",
		       i, *(lp_info->adc_table + i));
	}
	if (lightsensor_update_table(lpi) < 0)
		printk(KERN_ERR "[LS][CM3217 error] %s: update ls table fail\n",
		       __func__);

	mutex_unlock(&als_get_adc_mutex);

	D("[LS][CM3217] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_adc_table, 0664, ls_adc_table_show, ls_adc_table_store);

static uint8_t ALS_CONF1;

static ssize_t ls_conf1_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ALS_CONF1 = %x\n", ALS_CONF1);
}

static ssize_t ls_conf1_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "0x%x", &value);

	ALS_CONF1 = value;
	printk(KERN_INFO "[LS]set ALS_CONF1 = %x\n", ALS_CONF1);
	_cm3217_I2C_Write_Byte(ALS_W_CMD1_addr, ALS_CONF1);

	return count;
}

static DEVICE_ATTR(ls_conf1, 0664, ls_conf1_show, ls_conf1_store);

static uint8_t ALS_CONF2;
static ssize_t ls_conf2_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ALS_CONF2 = %x\n", ALS_CONF2);
}

static ssize_t ls_conf2_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "0x%x", &value);

	ALS_CONF2 = value;
	printk(KERN_INFO "[LS]set ALS_CONF2 = %x\n", ALS_CONF2);
	_cm3217_I2C_Write_Byte(ALS_W_CMD2_addr, ALS_CONF2);

	return count;
}

static DEVICE_ATTR(ls_conf2, 0664, ls_conf2_show, ls_conf2_store);

static ssize_t ls_fLevel_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fLevel = %d\n", fLevel);
}

static ssize_t ls_fLevel_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct cm3217_info *lpi = lp_info;
	int value = 0;

	sscanf(buf, "%d", &value);
	(value >= 0) ? (value = min(value, 10)) : (value = max(value, -1));
	fLevel = value;

	input_report_abs(lpi->ls_input_dev, ABS_MISC, fLevel);
	input_sync(lpi->ls_input_dev);

	printk(KERN_INFO "[LS]set fLevel = %d\n", fLevel);

	msleep(1000);
	fLevel = -1;

	return count;
}

static DEVICE_ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store);

static int lightsensor_setup(struct cm3217_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err("[LS][CM3217 error]%s: "
		       "could not allocate ls input device\n", __func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm3217-ls";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM3217 error]%s: "
		       "can not register ls input device\n", __func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][CM3217 error]%s: "
		       "can not register ls misc device\n", __func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int cm3217_setup(struct cm3217_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);

	ret = _cm3217_I2C_Write_Byte(ALS_W_CMD2_addr,
				     CM3217_ALS_WDM_DEFAULT_1
				     | CM3217_ALS_IT_2_T
				     | CM3217_ALS_BIT5_Default_1);
	if (ret < 0)
		return ret;

	ret = _cm3217_I2C_Write_Byte(ALS_W_CMD2_addr, CM3217_ALS_IT_100ms);
	msleep(10);

	return ret;
}

static void cm3217_early_suspend(struct early_suspend *h)
{
	struct cm3217_info *lpi = lp_info;

	D("[LS][CM3217] %s\n", __func__);

	lpi->als_enabled_before_suspend = lpi->als_enable;
	if (lpi->als_enable)
		lightsensor_disable(lpi);
}

static void cm3217_late_resume(struct early_suspend *h)
{
	struct cm3217_info *lpi = lp_info;

	D("[LS][CM3217] %s\n", __func__);

	if (lpi->als_enabled_before_suspend)
		lightsensor_enable(lpi);
}

static int cm3217_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3217_info *lpi;
	struct cm3217_platform_data *pdata;

	D("[CM3217] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm3217_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/* D("[CM3217] %s: client->irq = %d\n", __func__, client->irq); */

	lpi->i2c_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("[CM3217 error]%s: Assign platform_data error!!\n",
		       __func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}

	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);
	lpi->adc_table = pdata->levels;
	lpi->golden_adc = pdata->golden_adc;
	lpi->power = pdata->power;

	lpi->polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);

	lp_info = lpi;

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM3217 error]%s: lightsensor_setup error!!\n",
		       __func__);
		goto err_lightsensor_setup;
	}

	/* SET LUX STEP FACTOR HERE
	 * if adc raw value one step = 0.3 lux
	 * the following will set the factor 0.3 = 3/10
	 * and lpi->golden_adc = 3;
	 * set als_kadc = (ALS_CALIBRATED <<16) | 10; */

	als_kadc = (ALS_CALIBRATED << 16) | 10;
	lpi->golden_adc = 3;

	/* ls calibrate always set to 1 */
	lpi->ls_calibrate = 1;

	lightsensor_set_kvalue(lpi);
	ret = lightsensor_update_table(lpi);
	if (ret < 0) {
		pr_err("[LS][CM3217 error]%s: update ls table fail\n",
		       __func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm3217_wq");
	if (!lpi->lp_wq) {
		pr_err("[CM3217 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	ret = cm3217_setup(lpi);
	if (ret < 0) {
		pr_err("[ERR][CM3217 error]%s: cm3217_setup error!\n",
		       __func__);
		goto err_cm3217_setup;
	}

	lpi->cm3217_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm3217_class)) {
		ret = PTR_ERR(lpi->cm3217_class);
		lpi->cm3217_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm3217_class,
				    NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_kadc);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_gadc);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc_table);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf1);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf2);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_flevel);
	if (ret)
		goto err_create_ls_device_file;

	lpi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm3217_early_suspend;
	lpi->early_suspend.resume = cm3217_late_resume;
	register_early_suspend(&lpi->early_suspend);

	lpi->als_enable = 0;
	lpi->als_enabled_before_suspend = 0;
	D("[CM3217] %s: Probe success!\n", __func__);

	return ret;

err_create_ls_device_file:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm3217_class);
err_create_class:
err_cm3217_setup:
	destroy_workqueue(lpi->lp_wq);
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
	misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
err_platform_data_null:
	kfree(lpi);
	return ret;
}

static const struct i2c_device_id cm3217_i2c_id[] = {
	{CM3217_I2C_NAME, 0},
	{}
};

static struct i2c_driver cm3217_driver = {
	.id_table = cm3217_i2c_id,
	.probe = cm3217_probe,
	.driver = {
		.name = CM3217_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init cm3217_init(void)
{
	return i2c_add_driver(&cm3217_driver);
}

static void __exit cm3217_exit(void)
{
	i2c_del_driver(&cm3217_driver);
}

module_init(cm3217_init);
module_exit(cm3217_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3217 Driver");
MODULE_AUTHOR("Frank Hsieh <pengyueh@gmail.com>");
