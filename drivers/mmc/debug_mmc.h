#define MMC_DEBUG 1
#if MMC_DEBUG
#define MMC_DBG(fmt,args...) \
	do { printk(KERN_DEBUG "[mmc_debug]:%s:%d "fmt"\n", __func__, __LINE__, ##args); } \
	while (0)
#else
#define MMC_DBG(x...) do {} while (0)
#endif


#if 1
#define MMC_printk(fmt,args...) \
	do { printk(KERN_INFO "[mmc]:%s:%d "fmt"\n", __func__, __LINE__, ##args); } \
	while (0)
#else
#define MMC_prink(x...) do {} while (0)
#endif
