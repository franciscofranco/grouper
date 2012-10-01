#
# Makefile for industrial I/O Magnetometer sensors
#

obj-$(CONFIG_SENSORS_AK8975)	+= ak8975.o
obj-$(CONFIG_SENSORS_HMC5843)	+= hmc5843.o
obj-$(CONFIG_AMI306)            += inv-ami306.o

inv-ami306-objs := inv_ami306_core.o
inv-ami306-objs += inv_ami306_ring.o
inv-ami306-objs += inv_ami306_trigger.o

