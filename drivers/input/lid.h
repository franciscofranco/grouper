#ifndef _LID_H
#define _LID_H


// compiler option
#define LID_DEBUG	0

// debug utility
#if LID_DEBUG
#define LID_INFO(format, arg...)	\
	printk(KERN_INFO "hall_sensor: [%s] " format , __FUNCTION__ , ## arg)
#else
#define LID_INFO(format, arg...)
#endif

#define LID_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "hall_sensor: [%s] " format , __FUNCTION__ , ## arg)

#define LID_ERR(format, arg...)	\
	printk(KERN_ERR "hall_sensor: [%s] " format , __FUNCTION__ , ## arg)

//-----------------------------------------

#define DRIVER_LID     		"ASUS Hall Sensor Driver"
#define CONVERSION_TIME_MS		50

struct delayed_work lid_hall_sensor_work;
#endif