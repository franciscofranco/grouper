/*
 * include/linux/smb347-charger.h
 *
 * Battery charger driver interface for Summit smb347
 *
 * Copyright (C) 2012 NVIDIA Corporation
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
 *
 */

#ifndef __LINUX_smb347_CHARGER_H
#define __LINUX_smb347_CHARGER_H

#include <linux/regulator/machine.h>

#define SMB_DEBUG			0
#if SMB_DEBUG
#define SMB_INFO(format, arg...)	\
	printk(KERN_INFO "smb347_charger: [%s] " format , __FUNCTION__ , ## arg)
#else
#define SMB_INFO(format, arg...)
#endif

#define SMB_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "smb347_charger: [%s] " format , __FUNCTION__ , ## arg)

#define SMB_ERR(format, arg...)	\
	printk(KERN_ERR "smb347_charger: [%s] " format , __FUNCTION__ , ## arg)

/* Debug setting */
#define REG_POLLING_RATE	90

enum charging_states {
	idle,
	progress,
	completed,
	stopped,
};

enum charger_type {
	AC,
	USB,
};

enum cable_type {
	non_cable =0,
	usb_cable,
	TBD,
	ac_cable,
};

typedef void (*charging_callback_t)(enum charging_states state,
enum charger_type chrg_type, void *args);

struct smb347_charger {
	struct i2c_client	*client;
	struct device	*dev;
	struct delayed_work	inok_isr_work;
	struct delayed_work	stat_isr_work;
	struct delayed_work	regs_dump_work;
	struct mutex		cable_lock;
	void	*charger_cb_data;
	enum charging_states state;
	enum charger_type chrg_type;
	charging_callback_t	charger_cb;
	int suspend_ongoing;
};

int smb347_battery_online(void);
typedef void (*callback_t)(enum usb_otg_state otg_state, void *args);
/*
 * Register callback function for the client.
 * Used by fuel-gauge driver to get battery charging properties.
 */
extern int register_callback(charging_callback_t cb, void *args);
extern int register_otg_callback(callback_t cb, void *args);

#endif /*__LINUX_smb347_CHARGER_H */

