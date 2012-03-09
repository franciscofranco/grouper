/*
 * include/linux/smb349-charger.h
 *
 * Battery charger driver interface for Summit SMB349
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

#ifndef __LINUX_SMB349_CHARGER_H
#define __LINUX_SMB349_CHARGER_H

#include <linux/regulator/machine.h>

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

typedef void (*charging_callback_t)(enum charging_states state,
enum charger_type chrg_type, void *args);

struct smb349_charger {
	struct i2c_client	*client;
	struct device	*dev;
	void	*charger_cb_data;
	enum charging_states state;
	enum charger_type chrg_type;
	charging_callback_t	charger_cb;
};

int smb349_battery_online(void);
typedef void (*callback_t)(enum usb_otg_state to,
		enum usb_otg_state from, void *args);
/*
 * Register callback function for the client.
 * Used by fuel-gauge driver to get battery charging properties.
 */
extern int register_callback(charging_callback_t cb, void *args);
extern int register_otg_callback(callback_t cb, void *args);
extern int update_charger_status(void);

#endif /*__LINUX_SMB349_CHARGER_H */
