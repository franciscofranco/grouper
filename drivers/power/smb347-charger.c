/*
 * drivers/power/smb347-charger.c
 *
 * Battery charger driver for smb347 from summit microelectronics
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/smb347-charger.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"

#define smb347_CHARGE		0x00
#define smb347_CHRG_CRNTS	0x01
#define smb347_VRS_FUNC		0x02
#define smb347_FLOAT_VLTG	0x03
#define smb347_CHRG_CTRL	0x04
#define smb347_STAT_TIME_CTRL	0x05
#define smb347_PIN_CTRL		0x06
#define smb347_THERM_CTRL	0x07
#define smb347_SYSOK_USB3	0x08
#define smb347_CTRL_REG		0x09

#define smb347_OTG_TLIM_REG	0x0A
#define smb347_HRD_SFT_TEMP	0x0B
#define smb347_FAULT_INTR	0x0C
#define smb347_STS_INTR_1	0x0D
#define smb347_I2C_ADDR	0x0E
#define smb347_IN_CLTG_DET	0x10
#define smb347_STS_INTR_2	0x11

/* Command registers */
#define smb347_CMD_REG		0x30
#define smb347_CMD_REG_B	0x31
#define smb347_CMD_REG_c	0x33

/* Interrupt Status registers */
#define smb347_INTR_STS_A	0x35
#define smb347_INTR_STS_B	0x36
#define smb347_INTR_STS_C	0x37
#define smb347_INTR_STS_D	0x38
#define smb347_INTR_STS_E	0x39
#define smb347_INTR_STS_F	0x3A

/* Status registers */
#define smb347_STS_REG_A	0x3B
#define smb347_STS_REG_B	0x3C
#define smb347_STS_REG_C	0x3D
#define smb347_STS_REG_D	0x3E
#define smb347_STS_REG_E	0x3F

#define smb347_ENABLE_WRITE	1
#define smb347_DISABLE_WRITE	0
#define ENABLE_WRT_ACCESS	0x80
#define ENABLE_APSD		0x04
#define HC_MODE		0x01
#define USB_5_9_CUR		0x02
#define PIN_CTRL		0x10
#define THERM_CTRL		0x10
#define BATTERY_MISSING		0x10
#define CHARGING		0x06
#define DEDICATED_CHARGER	0x02
#define CHRG_DOWNSTRM_PORT	0x04
#define ENABLE_CHARGE		0x02
#define ENABLE_CHARGER		1
#define DISABLE_CHARGER		0
#define USBIN		0x80
#define APSD_OK		0x08
#define APSD_RESULT		0x07
#define APSD_CDP		0x01
#define APSD_DCP		0x02
#define APSD_OTHER		0x03
#define APSD_SDP		0x04
#define USB_30		0x20

/* Functions declaration */
static int smb347_configure_charger(struct i2c_client *client, int value);
static int smb347_configure_interrupts(struct i2c_client *client);
extern unsigned int grouper_query_pcba_revision();
extern int battery_callback(unsigned usb_cable_state);
/* Enable or disable the callback for the battery driver. */
#define TOUCH_CALLBACK_ENABLED 1
#ifdef TOUCH_CALLBACK_ENABLED
extern void touch_callback(unsigned cable_status);
#endif

static ssize_t smb347_reg_show(struct device *dev, struct device_attribute *attr, char *buf);

/* Global variables */
static struct smb347_charger *charger;
static struct workqueue_struct *smb347_wq;

/* Sysfs interface */
static DEVICE_ATTR(reg_status, S_IWUSR | S_IRUGO, smb347_reg_show, NULL);

static struct attribute *smb347_attributes[] = {
	&dev_attr_reg_status.attr,
NULL
};

static const struct attribute_group smb347_group = {
	.attrs = smb347_attributes,
};

static int smb347_read(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_write(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb347_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb347_write(client, reg, retval | value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int smb347_clear_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb347_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb347_write(client, reg, retval & (~value));
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int smb347_volatile_writes(struct i2c_client *client, uint8_t value)
{
	int ret = 0;

	if (value == smb347_ENABLE_WRITE) {
		/* Enable volatile write to config registers */
		ret = smb347_update_reg(client, smb347_CMD_REG,
						ENABLE_WRT_ACCESS);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG);
			return ret;
		}
	} else {
		ret = smb347_read(client, smb347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}

		ret = smb347_write(client, smb347_CMD_REG, ret & (~(1<<7)));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

static void smb347_clear_interrupts(struct i2c_client *client)
{
	uint8_t val, buf[6];

	val = i2c_smbus_read_i2c_block_data(client, smb347_INTR_STS_A, 6, buf);
	if (val < 0)
		dev_err(&client->dev, "%s(): Failed in clearing interrupts\n",
								__func__);
}

static int smb347_configure_otg(struct i2c_client *client, int enable)
{
	int ret = 0;

	/*Enable volatile writes to registers*/
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring otg..\n",
								__func__);
		goto error;
	}

	if (enable) {

		/* Configure INOK to be active high */
		ret = smb347_update_reg(client, smb347_SYSOK_USB3, 0x01);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		/* Change "OTG output current limit" to 250mA */
		ret = smb347_read(client, smb347_OTG_TLIM_REG);
	       if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
	       }
	       ret = smb347_write(client, smb347_OTG_TLIM_REG, (ret & (~(1<<3))));
	       if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
	       }

		/* Enable OTG */
	       ret = smb347_update_reg(client, smb347_CMD_REG, 0x10);
	       if (ret < 0) {
		       dev_err(&client->dev, "%s: Failed in writing register"
				"0x%02x\n", __func__, smb347_CMD_REG);
			goto error;
	       }

		/* Change "OTG output current limit" from 250mA to 750mA */
		ret = smb347_update_reg(client, smb347_OTG_TLIM_REG, 0x08);
	       if (ret < 0) {
		       dev_err(&client->dev, "%s: Failed in writing register"
				"0x%02x\n", __func__, smb347_OTG_TLIM_REG);
			goto error;
	       }

	} else {
	       /* Disable OTG */
	       ret = smb347_read(client, smb347_CMD_REG);
	       if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
	       }

	       ret = smb347_write(client, smb347_CMD_REG, (ret & (~(1<<4))));
	       if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
	       }

		/* Configure INOK to be active low */
		ret = smb347_read(client, smb347_SYSOK_USB3);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb347_write(client, smb347_SYSOK_USB3, (ret & (~(1))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}

	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
	       goto error;
	}
error:
	return ret;
}

static int smb347_configure_charger(struct i2c_client *client, int value)
{
	int ret = 0;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if (value) {
		 /* Enable charging */
		ret = smb347_update_reg(client, smb347_CMD_REG, ENABLE_CHARGE);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x\n", __func__, smb347_CMD_REG);
			goto error;
		}

		/* Configure THERM ctrl */
		/*
		ret = smb347_update_reg(client, smb347_THERM_CTRL, THERM_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
		*/
	} else {
		/* Disable charging */
		ret = smb347_read(client, smb347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb347_write(client, smb347_CMD_REG, (ret & (~(1<<1))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}
	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
error:
	return ret;
}

static irqreturn_t smb347_inok_isr(int irq, void *dev_id)
{
	struct smb347_charger *smb = dev_id;

		disable_irq_nosync(irq);
		//printk("interrupt: %s +, disable irq=%d\n",__func__, irq);
		queue_delayed_work(smb347_wq, &smb->inok_isr_work, 0.6*HZ);
		//printk("interrupt: %s  -\n",__func__);
	return IRQ_HANDLED;
}

static int smb347_inok_irq(struct smb347_charger *smb)
{
	int err = 0 ;
	unsigned gpio = TEGRA_GPIO_PV1;		//Tegra3: SMB347_ACOK# <--> smb347: INOK/SYSOK
	unsigned irq_num = gpio_to_irq(gpio);

	err = gpio_request(gpio, "smb347_inok");
	if (err) {
		printk("gpio %d request failed \n", gpio);
		goto err1;
	}

	tegra_gpio_enable(gpio);

	err = gpio_direction_input(gpio);
	if (err) {
		printk("gpio %d unavaliable for input \n", gpio);
		goto err2;
	}

	err = request_irq(irq_num, smb347_inok_isr, IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING,
		"smb347_inok", smb);
	if (err < 0) {
		printk("%s irq %d request failed \n","smb347_inok", irq_num);
		goto err2 ;
	}
	printk("GPIO pin irq %d requested ok, smb347_INOK = %s\n", irq_num, gpio_get_value(gpio)? "H":"L");

	disable_irq(irq_num);
	queue_delayed_work(smb347_wq, &charger->inok_isr_work, 0.5*HZ);

	return 0 ;

err2:
	gpio_free(gpio);
err1:
	return err;
}

int register_callback(charging_callback_t cb, void *args)
{
	struct smb347_charger *charger_data = charger;
	if (!charger_data)
		return -ENODEV;

	charger_data->charger_cb = cb;
	charger_data->charger_cb_data = args;
	return 0;
}
EXPORT_SYMBOL_GPL(register_callback);

int smb347_hc_mode_callback(bool enable, int cur)
{
	struct i2c_client *client = charger->client;
	u8 ret = 0;

	printk("smb347_hc_mode_callback+\n");

	if (charger->suspend_ongoing)
		return 0;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
							__func__);
		goto error;
	}

	if(enable) {
		/* Force switch to HC mode */
		ret = smb347_update_reg(client, smb347_CMD_REG_B,
						HC_MODE);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG_B);
			return ret;
		}

		/* Change to i2c register control */
		ret = smb347_clear_reg(client, smb347_PIN_CTRL, PIN_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_PIN_CTRL);
			return ret;
		}
	}
	else
	{
		/* USB 2.0 input current limit (ICL) */
		ret = smb347_clear_reg(client, smb347_SYSOK_USB3, USB_30);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_SYSOK_USB3);
			return ret;
		}

		/* Switch back to USB mode */
		ret = smb347_clear_reg(client, smb347_CMD_REG_B, HC_MODE);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG_B);
			return ret;
		}

		if(cur) {
			/* USB 500mA */
			ret = smb347_update_reg(client, smb347_CMD_REG_B, USB_5_9_CUR);
			if (ret < 0) {
				dev_err(&client->dev, "%s(): Failed in writing"
					"register 0x%02x\n", __func__, smb347_CMD_REG_B);
				return ret;
			}
		} else {
			/* USB 100mA */
			ret = smb347_clear_reg(client, smb347_CMD_REG_B, USB_5_9_CUR);
			if (ret < 0) {
				dev_err(&client->dev, "%s(): Failed in writing"
					"register 0x%02x\n", __func__, smb347_CMD_REG_B);
				return ret;
			}
		}

		/* Disable auto power source detection (APSD) */
		ret = smb347_clear_reg(client, smb347_CHRG_CTRL, ENABLE_APSD);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CHRG_CTRL);
			return ret;
		}

		/* Change to i2c register control */
		ret = smb347_clear_reg(client, smb347_PIN_CTRL, PIN_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_PIN_CTRL);
			return ret;
		}
	}

	 /* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	printk("smb347_hc_mode_callback-\n");
	return ret;

error:
	return ret;
}
EXPORT_SYMBOL_GPL(smb347_hc_mode_callback);

int smb347_battery_online(void)
{
	int val, ret;
	struct i2c_client *client = charger->client;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	val = smb347_read(client, smb347_INTR_STS_B);
	if (val < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
				"0x%02x\n", __func__, smb347_INTR_STS_B);
		return val;
	}
	if (val & BATTERY_MISSING)
		return 0;
	else
		return 1;

	 /* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	return ret;
}

static int smb347_configure_interrupts(struct i2c_client *client)
{
	int ret = 0;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
	/* Setting: Fault assert STAT IRQ */
	ret = smb347_update_reg(client, smb347_FAULT_INTR, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing register"
				"0x%02x\n", __func__, smb347_CMD_REG);
		goto error;
	}
	/* Setting: Status assert STAT IRQ */
	ret = smb347_update_reg(client, smb347_STS_INTR_1, 0x14);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		goto error;
	}

	 /* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	return ret;
}

static void smb347_otg_status(enum usb_otg_state to, enum usb_otg_state from, void *data)
{
	struct i2c_client *client = charger->client;
	int ret;

	if ((from == OTG_STATE_A_SUSPEND) && (to == OTG_STATE_A_HOST)) {

		/* configure charger */
		ret = smb347_configure_charger(client, 0);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);

		/* ENABLE OTG */
		ret = smb347_configure_otg(client, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);

	} else if ((from == OTG_STATE_A_HOST) && (to == OTG_STATE_A_SUSPEND)) {

		/* Disable OTG */
		ret = smb347_configure_otg(client, 0);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);

		/* configure charger */
		ret = smb347_configure_charger(client, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);

		/*
		ret = smb347_configure_interrupts(client);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
						"otg..\n", __func__);
		*/
	}
}

/* workqueue function */
static int cable_type_detect(void)
{
	struct i2c_client *client = charger->client;
	u8 retval;
	int  success = 0;
	int gpio = TEGRA_GPIO_PV1;

	if(grouper_query_pcba_revision() <= 0x02)
		return 0;

	mutex_lock(&charger->cable_lock);

	if (gpio_get_value(gpio)) {
			printk("INOK=H\n");
			success = battery_callback(non_cable);
#ifdef TOUCH_CALLBACK_ENABLED
                   touch_callback(non_cable);
#endif

	} else {
			printk("INOK=L\n");

			/* cable type dection */
			retval = smb347_read(client, smb347_STS_REG_E);
			SMB_NOTICE("Reg3F : 0x%02x\n", retval);
			if(retval & USBIN) {	//USBIN
					retval = smb347_read(client, smb347_STS_REG_D);
					SMB_NOTICE("Reg3E : 0x%02x\n", retval);
				if(retval & APSD_OK) {	//APSD completed
						retval &= APSD_RESULT;
					if(retval == APSD_CDP) {	//APSD resulted
							printk("Cable: CDP\n");
							success = battery_callback(ac_cable);
#ifdef TOUCH_CALLBACK_ENABLED
                                             touch_callback(ac_cable);
#endif 
					} else if(retval == APSD_DCP) {
							printk("Cable: DCP\n");
							success = battery_callback(ac_cable);
#ifdef TOUCH_CALLBACK_ENABLED
                                             touch_callback(ac_cable);
#endif 

					} else if(retval == APSD_OTHER) {
							printk("Cable: OTHER\n");
					} else if(retval == APSD_SDP) {
							printk("Cable: SDP\n");
							success = battery_callback(usb_cable);
#ifdef TOUCH_CALLBACK_ENABLED
                                             touch_callback(usb_cable);
#endif 

					} else
							printk("Unkown Plug In Cable type !\n");
				}else
					printk("APSD not completed\n");
			}
			else
			{
					printk("USBIN=0\n");
			}
	}

	mutex_unlock(&charger->cable_lock);

	return success;
}

static void inok_isr_work_function(struct work_struct *dat)
{
	struct i2c_client *client = charger->client;

	int gpio = TEGRA_GPIO_PV1;
	int irq = gpio_to_irq(gpio);

	cancel_delayed_work(&charger->inok_isr_work);
	cable_type_detect();

	printk("inok_isr_work_function -\n");
	smb347_clear_interrupts(client);
	enable_irq(irq);
}

/*static void regs_dump_work_func(struct work_struct *dat)
{
	struct i2c_client *client = charger->client;
	uint8_t config_reg[2], cmd_reg[2], status_reg[2];
	int ret = 0;

	ret += i2c_smbus_read_i2c_block_data(client, smb347_PIN_CTRL, 3, config_reg)
	     + i2c_smbus_read_i2c_block_data(client, smb347_CMD_REG, 2, cmd_reg)
	     + i2c_smbus_read_i2c_block_data(client, smb347_STS_REG_C, 3, status_reg);

	if (ret < 0)
		SMB_ERR("failed to read charger reg !\n");

	SMB_NOTICE("\n"
		"Reg[06h]=0x%02x\n"
		"Reg[08h]=0x%02x\n"
		"Reg[30h]=0x%02x\n"
		"Reg[31h]=0x%02x\n"
		"Reg[3dh]=0x%02x\n"
		"Reg[3eh]=0x%02x\n"
		"Reg[3fh]=0x%02x\n",
		config_reg[0],
		config_reg[2],
		cmd_reg[0],
		cmd_reg[1],
		status_reg[0],
		status_reg[1],
		status_reg[2]);

	// Schedule next polling
	queue_delayed_work(smb347_wq, &charger->regs_dump_work, REG_POLLING_RATE*HZ);
}*/

/* Sysfs function */
static ssize_t smb347_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = charger->client;
	uint8_t config_reg[14], cmd_reg[1], status_reg[10];
	int i, ret = 0;

	ret += i2c_smbus_read_i2c_block_data(client, smb347_CHARGE, 15, config_reg)
	     + i2c_smbus_read_i2c_block_data(client, smb347_CMD_REG, 2, cmd_reg)
	     + i2c_smbus_read_i2c_block_data(client, smb347_INTR_STS_A, 11, status_reg);

	if (ret < 0)
		SMB_ERR("failed to read charger reg !\n");

	printk("smb347 Registers\n");
	printk("------------------\n");
	for(i=0;i<=14;i++)
		printk("Reg[%02xh]=0x%02x\n", i, config_reg[i]);
	for(i=0;i<=1;i++)
		printk("Reg[%02xh]=0x%02x\n", 48+i, cmd_reg[i]);
	for(i=0;i<=10;i++)
		printk("Reg[%02xh]=0x%02x\n", 53+i, status_reg[i]);

	return sprintf(buf, "Reg[06h]=0x%02x\n"
		"Reg[08h]=0x%02x\n"
		"Reg[30h]=0x%02x\n"
		"Reg[31h]=0x%02x\n"
		"Reg[3dh]=0x%02x\n"
		"Reg[3eh]=0x%02x\n"
		"Reg[3fh]=0x%02x\n",
		config_reg[6],
		config_reg[8],
		cmd_reg[0],
		cmd_reg[1],
		status_reg[8],
		status_reg[9],
		status_reg[10]);

}

static void smb347_default_setback(void)
{
	struct i2c_client *client = charger->client;
	int err;

	if(grouper_query_pcba_revision() > 0x02) {
		/* Enable volatile writes to registers */
		err = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
		if (err < 0) {
			dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		}
		err = smb347_update_reg(client, smb347_PIN_CTRL, PIN_CTRL);
		if (err < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, err);
		}
		err = smb347_update_reg(client, smb347_CHRG_CTRL, ENABLE_APSD);
		if (err < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, err);
		}
		 /* Disable volatile writes to registers */
		err = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
		if (err < 0) {
			dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		}
		printk("grouper_query_pcba_revision=0x%02x\n",
			grouper_query_pcba_revision());
	}
}

static int __devinit smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret, irq_num, i;
	uint8_t val, buf[15];

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->client = client;
	charger->dev = &client->dev;
	i2c_set_clientdata(client, charger);

	/* Restore default setting: APSD Enable & 5/1/HC mode Pin control */
	smb347_default_setback();

	ret = sysfs_create_group(&client->dev.kobj, &smb347_group);
	if (ret) {
		dev_err(&client->dev, "smb347_probe: unable to create the sysfs\n");
	}

	mutex_init(&charger->cable_lock);

	smb347_wq = create_singlethread_workqueue("smb347_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&charger->inok_isr_work, inok_isr_work_function);
	//INIT_DELAYED_WORK(&charger->regs_dump_work, regs_dump_work_func);

	ret = smb347_inok_irq(charger);
	if (ret) {
		dev_err(&client->dev, "%s(): Failed in requesting ACOK# pin isr\n",
				__func__);
		goto error;
	}

	//queue_delayed_work(smb347_wq, &charger->regs_dump_work, 30*HZ);

	cable_type_detect();

	ret = register_otg_callback(smb347_otg_status, charger);
	if (ret < 0)
		goto error;

	return 0;
error:
	kfree(charger);
	return ret;
}

static int __devexit smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *charger = i2c_get_clientdata(client);

	kfree(charger);
	return 0;
}

static int smb347_suspend(struct i2c_client *client)
{
	charger->suspend_ongoing = 1;
	//cancel_delayed_work_sync(&charger->regs_dump_work);
	printk("smb347_suspend+\n");
	flush_workqueue(smb347_wq);
	printk("smb347_suspend-\n");
	return 0;
}

static int smb347_resume(struct i2c_client *client)
{
	charger->suspend_ongoing = 0;
	//cancel_delayed_work(&charger->regs_dump_work);
	//queue_delayed_work(smb347_wq, &charger->regs_dump_work, 15*HZ);

	printk("smb347_resume+\n");
	cable_type_detect();
	printk("smb347_resume-\n");
	return 0;
}


static int smb347_shutdown(struct i2c_client *client)
{
	int ret;
	printk("smb347_shutdown+\n");

	/* Disable OTG */
	ret = smb347_configure_otg(client, 0);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in configuring"
			"otg..\n", __func__);

	/* configure charger */
	ret = smb347_configure_charger(client, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in configuring"
			"otg..\n", __func__);

	printk("smb347_shutdown-\n");
	return 0;
}

static const struct i2c_device_id smb347_id[] = {
	{ "smb347", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static struct i2c_driver smb347_i2c_driver = {
	.driver	= {
		.name	= "smb347",
	},
	.probe		= smb347_probe,
	.remove		= __devexit_p(smb347_remove),
	.suspend 		= smb347_suspend,
	.resume 		= smb347_resume,
	.shutdown	= smb347_shutdown,
	.id_table	= smb347_id,
};

static int __init smb347_init(void)
{
	return i2c_add_driver(&smb347_i2c_driver);
}
module_init(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_i2c_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_DESCRIPTION("smb347 Battery-Charger");
MODULE_LICENSE("GPL");

