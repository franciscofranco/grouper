#define BATTERY_IOC_MAGIC	0xF9
#define BATTERY_IOC_MAXNR	15
#define BATTERY_POLLING_DATA _IOR(BATTERY_IOC_MAGIC, 1,int) //magic, ioctl index, parameter
#define BATTERY_START_POLLING _IOR(BATTERY_IOC_MAGIC, 2,int) //magic, ioctl index, parameter
#define BATTERY_POWER_KEY _IOR(BATTERY_IOC_MAGIC, 3,int*) //magic, ioctl index, parameter
#define BATTERY_DOCKING_STATUS  _IOR(BATTERY_IOC_MAGIC, 4,int*)
#define BATTERY_ENABLE_CHARGER     _IOR(BATTERY_IOC_MAGIC, 5,int)
#define BATTERY_ENABLE_BACKLIGHT     _IOR(BATTERY_IOC_MAGIC, 6,int)
#define BATTERY_USB_STATUS  	          _IOR(BATTERY_IOC_MAGIC, 7,int*)
#define BATTERY_REBOOT_TEST_TOOL      _IOR(BATTERY_IOC_MAGIC, 8,int)
#define BATTERY_STATUS 	                         _IOR(BATTERY_IOC_MAGIC, 9,int*)
#define BOOT_REASON	                         _IOR(BATTERY_IOC_MAGIC, 10,int*)

#define DACKING_INSTERTION (1<0)
#define DACKING_BATTERY_HIGHER_10  (1<1)

#define TEST_END (0)
#define START_NORMAL (1)
#define START_HEAVY (2)
#define IOCTL_ERROR (-1)
extern unsigned int boot_reason;
static void battery_strees_test(struct work_struct *work)
{
	int ret=0;
	int rt_value;
       struct bq27541_device_info *battery_device =container_of(work, struct bq27541_device_info, battery_stress_test.work);

	ret = bq27541_smbus_read_data(REG_STATUS,0,&rt_value);
	if (ret < 0) {
		printk("battery_strees_test: i2c read for REG_STATUS failed  ret=%d\n", ret);
	}
	queue_delayed_work(battery_work_queue , &battery_device->battery_stress_test, 2*HZ);
}

long battery_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
       int fc=0;
	int battery_status=0;
	if (_IOC_TYPE(cmd) == BATTERY_IOC_MAGIC ){
	     //printk("  battery_ioctl vaild magic \n");
	}
	else	
		return -ENOTTY;
	switch(cmd)
	{
	       case BATTERY_POLLING_DATA:
		    if ((arg==START_NORMAL)||(arg==START_HEAVY)){
				 printk(" BATTERY:  battery stress test start (%s)\n",(arg==START_NORMAL)?"normal":"heavy");
				 queue_delayed_work(battery_work_queue , &bq27541_device->battery_stress_test, 2*HZ);
			}
		else{
				 printk(" BATTERY:  battery stress test end\n");
				 cancel_delayed_work_sync(&bq27541_device->battery_stress_test);
			}
		break;
               case BATTERY_REBOOT_TEST_TOOL:
			break;
		case BOOT_REASON:
			(*(int*)arg)=boot_reason;
			printk(" BATTERY: BOOT_REASON=%u\n",(*(int*)arg));
			break;
	  default:  /* redundant, as cmd was checked against MAXNR */
	           printk(" BATTERY: unknow i2c  stress test  command cmd=%x arg=%lu\n",cmd,arg);
		    printk(" BATTERY_POLLING_DATA=%x\n",BATTERY_POLLING_DATA);
		return -ENOTTY;
	}
return 0;
}
int battery_open(struct inode *inode, struct file *filp)
{
	return 0;
}
struct file_operations battery_fops = {
	.owner =    THIS_MODULE,
	.unlocked_ioctl =   battery_ioctl,
	.open =   battery_open,
};

