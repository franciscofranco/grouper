/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2012 Nvidia Cooperation
 *  Chandler Zhang <chazhang@nvidia.com>
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/max17048_battery.h>

#define MAX17048_VCELL		0x02
#define MAX17048_SOC		0x04
#define MAX17048_VER		0x08
#define MAX17048_HIBRT		0x0A
#define MAX17048_CONFIG		0x0C
#define MAX17048_OCV		0x0E
#define MAX17048_VLRT		0x14
#define MAX17048_VRESET		0x18
#define MAX17048_STATUS		0x1A
#define MAX17048_UNLOCK		0x3E
#define MAX17048_TABLE		0x40
#define MAX17048_RCOMPSEG1	0x80
#define MAX17048_RCOMPSEG2	0x90
#define MAX17048_CMD		0xFF
#define MAX17048_UNLOCK_VALUE	0x4a57
#define MAX17048_RESET_VALUE	0x5400
#define MAX17048_DELAY		1000
#define MAX17048_BATTERY_FULL	100
#define MAX17048_BATTERY_LOW	15
#define MAX17048_VERSION_NO	0x11

struct max17048_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct power_supply		ac;
	struct power_supply		usb;
	struct max17048_battery_model	*model_data;

	/* State Of Connect */
	int ac_online;
	int usb_online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;

	int lasttime_vcell;
	int lasttime_soc;
	int lasttime_status;
};

uint8_t max17048_custom_data[] = {
	0xAA, 0x00, 0xB1, 0xF0, 0xB7, 0xE0, 0xB9, 0x60, 0xBB, 0x80,
	0xBC, 0x40, 0xBD, 0x30, 0xBD, 0x50, 0xBD, 0xF0, 0xBE, 0x40,
	0xBF, 0xD0, 0xC0, 0x90, 0xC4, 0x30, 0xC7, 0xC0, 0xCA, 0x60,
	0xCF, 0x30, 0x01, 0x20, 0x09, 0xC0, 0x1F, 0xC0, 0x2B, 0xE0,
	0x4F, 0xC0, 0x30, 0x00, 0x47, 0x80, 0x4F, 0xE0, 0x77, 0x00,
	0x15, 0x60, 0x46, 0x20, 0x13, 0x80, 0x1A, 0x60, 0x12, 0x20,
	0x14, 0xA0, 0x14, 0xA0};

static int max17048_write_word(struct i2c_client *client, int reg, u16 value)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x err %d\n", __func__, reg, ret);

	return ret;
}

static int max17048_read_word(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
					"0x%02x err %d\n", __func__, reg, ret);
		return ret;
	} else {
		ret = (int)swab16((uint16_t)(ret & 0x0000ffff));
		return ret;
	}
}

static int max17048_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->capacity_level;
		break;
	default:
	return -EINVAL;
	}
	return 0;
}

static int max17048_ac_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, ac);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->ac_online;
	else
		return -EINVAL;

	return 0;
}

static int max17048_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
					struct max17048_chip, usb);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->usb_online;
	else
		return -EINVAL;

	return 0;
}

static void max17048_get_vcell(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int vcell;

	vcell = max17048_read_word(client, MAX17048_VCELL);
	if (vcell < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, vcell);
	else
		chip->vcell = (uint16_t)vcell;
}

static void max17048_get_soc(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int soc;

	soc = max17048_read_word(client, MAX17048_SOC);
	if (soc < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, soc);
	else
		chip->soc = (uint16_t)soc >> 9;

	if (chip->soc > MAX17048_BATTERY_FULL) {
		chip->soc = MAX17048_BATTERY_FULL;
		chip->status = POWER_SUPPLY_STATUS_FULL;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (chip->soc < MAX17048_BATTERY_LOW) {
		chip->health = POWER_SUPPLY_HEALTH_DEAD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}

static uint16_t max17048_get_version(struct i2c_client *client)
{
	return swab16(i2c_smbus_read_word_data(client, MAX17048_VER));
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;

	chip = container_of(work, struct max17048_chip, work.work);

	max17048_get_vcell(chip->client);
	max17048_get_soc(chip->client);

	if (chip->vcell != chip->lasttime_vcell ||
		chip->soc != chip->lasttime_soc ||
		chip->status !=	chip->lasttime_status) {

		chip->lasttime_vcell = chip->vcell;
		chip->lasttime_soc = chip->soc;

		power_supply_changed(&chip->battery);
	}
	schedule_delayed_work(&chip->work, MAX17048_DELAY);
}

static void max17048_battery_status(enum charging_states status,
				enum charger_type chrg_type, void *data)
{
	struct max17048_chip *chip = data;

	chip->ac_online = 0;
	chip->usb_online = 0;

	if (chrg_type == AC)
		chip->ac_online = 1;
	else if (chrg_type == USB)
		chip->usb_online = 1;

	if (status == progress)
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
	else
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;


	power_supply_changed(&chip->battery);
	power_supply_changed(&chip->usb);
	power_supply_changed(&chip->ac);
}

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static enum power_supply_property max17048_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property max17048_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int max17048_write_rcomp_seg(struct i2c_client *client,
						uint16_t rcomp_seg)
{
	uint8_t rs1, rs2;
	int ret;

	rs2 = rcomp_seg | 0x00FF;
	rs1 = rcomp_seg >> 8;
	uint8_t rcomp_seg_table[16] = { rs1, rs2, rs1, rs2,
					rs1, rs2, rs1, rs2,
					rs1, rs2, rs1, rs2,
					rs1, rs2, rs1, rs2};

	ret = i2c_smbus_write_i2c_block_data(client, MAX17048_RCOMPSEG1,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	ret = i2c_smbus_write_i2c_block_data(client, MAX17048_RCOMPSEG2,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int max17048_load_model_data(struct max17048_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->model_data;
	uint16_t soc_tst, ocv;
	int i, ret = 0;

	/* read OCV */
	ret = max17048_read_word(client, MAX17048_OCV);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	ocv = (uint16_t)ret;
	if (ocv == 0xffff) {
		dev_err(&client->dev, "%s: Failed in unlocking"
					"max17048 err: %d\n", __func__, ocv);
		return -1;
	}

	/* write custom model data */
	for (i = 0; i < 4; i += 1) {
		if (i2c_smbus_write_i2c_block_data(client,
			(MAX17048_TABLE+i*16), 16,
				&max17048_custom_data[i*0x10]) < 0) {
			dev_err(&client->dev, "%s: error writing model data:\n",
								__func__);
			return -1;
		}
	}

	/* Write OCV Test value */
	ret = max17048_write_word(client, MAX17048_OCV, mdata->ocvtest);
	if (ret < 0)
		return ret;

	ret = max17048_write_rcomp_seg(client, mdata->rcomp_seg);
	if (ret < 0)
		return ret;

	/* Disable hibernate */
	ret = max17048_write_word(client, MAX17048_HIBRT, 0x0000);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Delay between 150ms to 600ms */
	mdelay(200);

	/* Read SOC Register and compare to expected result */
	ret = max17048_read_word(client, MAX17048_SOC);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	soc_tst = (uint16_t)ret;
	if (!((soc_tst >> 8) >= mdata->soccheck_A &&
				(soc_tst >> 8) <=  mdata->soccheck_B)) {
		dev_err(&client->dev, "%s: soc comparison failed %d\n",
					__func__, ret);
		return ret;
	} else {
		dev_info(&client->dev, "MAX17048 Custom data"
						" loading successfull\n");
	}

	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
					MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* Restore OCV */
	ret = max17048_write_word(client, MAX17048_OCV, ocv);
	if (ret < 0)
		return ret;

	return ret;
}

static int max17048_initialize(struct max17048_chip *chip)
{
	uint8_t ret, config, status;
	uint16_t wrt_status;
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->model_data;

	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* load model data */
	ret = max17048_load_model_data(chip);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	if (mdata->bits == 19)
		config = 32 - (mdata->alert_threshold * 2);
	else if (mdata->bits == 18)
		config = 32 - mdata->alert_threshold;

	config = mdata->one_percent_alerts | config;

	ret = max17048_write_word(client, MAX17048_CONFIG,
			((mdata->rcomp << 8) | config));
	if (ret < 0)
		return ret;

	/* Voltage Alert configuration */
	ret = max17048_write_word(client, MAX17048_VLRT, mdata->valert);
	if (ret < 0)
		return ret;

	/* Hibernate configuration */
	ret = max17048_write_word(client, MAX17048_HIBRT, mdata->hibernate);
	if (ret < 0)
		return ret;

	ret = max17048_write_word(client, MAX17048_VRESET, mdata->vreset);
	if (ret < 0)
		return ret;

	/* clears the reset indicator */
	ret = max17048_read_word(client, MAX17048_STATUS);
	if (ret < 0)
		return ret;

	/* Sets the EnVR bit if selected */
	status = (ret & 0xFE) | mdata->alert_on_reset;
	wrt_status = status << 8;

	ret = max17048_write_word(client, MAX17048_STATUS, wrt_status);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Add delay */
	mdelay(200);
	return 0;
}

static int __devinit max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max17048_chip *chip;
	int ret;
	uint16_t version;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->model_data = client->dev.platform_data;
	chip->ac_online = 0;
	chip->usb_online = 0;

	i2c_set_clientdata(client, chip);

	version = max17048_get_version(client);
	if (version != MAX17048_VERSION_NO) {
		ret = -ENODEV;
		goto error2;
	}
	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver 0x%x\n", version);

	ret = max17048_initialize(chip);
	if (ret < 0) {
		dev_err(&client->dev, "Error: Initializing fuel-gauge\n");
		goto error2;
	}

	ret = register_callback(max17048_battery_status, chip);
	if (ret < 0)
		goto error2;

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17048_get_property;
	chip->battery.properties	= max17048_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17048_battery_props);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error2;
	}

	chip->ac.name		= "maxim-ac";
	chip->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	chip->ac.get_property	= max17048_ac_get_property;
	chip->ac.properties	= max17048_ac_props;
	chip->ac.num_properties	= ARRAY_SIZE(max17048_ac_props);

	ret = power_supply_register(&client->dev, &chip->ac);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error1;
	}

	chip->usb.name		= "maxim-usb";
	chip->usb.type		= POWER_SUPPLY_TYPE_USB;
	chip->usb.get_property	= max17048_usb_get_property;
	chip->usb.properties	= max17048_usb_props;
	chip->usb.num_properties	= ARRAY_SIZE(max17048_usb_props);

	ret = power_supply_register(&client->dev, &chip->usb);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17048_work);
	schedule_delayed_work(&chip->work, MAX17048_DELAY);

	ret = update_charger_status();
	if (ret) {
		dev_err(&client->dev, "failed: update_charger_status\n");
		goto error;
	}

	return 0;
error:
	power_supply_unregister(&chip->ac);
error1:
	power_supply_unregister(&chip->battery);
error2:
	kfree(chip);
	return ret;
}

static int __devexit max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	power_supply_unregister(&chip->usb);
	power_supply_unregister(&chip->ac);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17048_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max17048_resume(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, MAX17048_DELAY);
	return 0;
}

#else

#define max17048_suspend NULL
#define max17048_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
	},
	.probe		= max17048_probe,
	.remove		= __devexit_p(max17048_remove),
	.suspend	= max17048_suspend,
	.resume		= max17048_resume,
	.id_table	= max17048_id,
};

static int __init max17048_init(void)
{
	return i2c_add_driver(&max17048_i2c_driver);
}
module_init(max17048_init);

static void __exit max17048_exit(void)
{
	i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);

MODULE_AUTHOR("Chandler Zhang <chazhang@nvidia.com>");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
