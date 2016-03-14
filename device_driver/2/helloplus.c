/* 
 *  helloplus.c - Hello with IOCTL and timers
 */

#include <linux/module.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/kd.h>
#include <linux/vt_kern.h>

MODULE_DESCRIPTION("Example driver illustrating the use of Timers and IOCTL.");
MODULE_AUTHOR("Your Name Here");
MODULE_LICENSE("GPL");

static struct workqueue_struct *my_wq;
typedef struct {
	struct delayed_work my_work;
	int delay;
	int led;
	int led_status;
} my_work_t;
my_work_t *work;

struct tty_driver *my_driver;
char helloplustatus = 0;

#define MAGIC 'z'
#define GET_DELAY      _IOR(MAGIC, 1, int *)
#define SET_DELAY      _IOW(MAGIC, 2, int *)
#define SET_LED		   _IOW(MAGIC, 3, int *)

/**
Don't need it if you want major number assigned to your module dynamically 
#define DEV_MAJOR       250
#define DEV_MINOR       5
#define HELLOPLUS "helloplus"
*/

#define ALL_LEDS_OFF 0x00
#define ALL_LEDS_ON 0x07
#define CAP_LED_ON 0x04
#define SCROLL_LED_ON 0x01
#define NUM_LED_ON 0x02
#define INITIAL_SECS 	HZ
static unsigned long secs2hello = INITIAL_SECS;
static unsigned long led2hello = ALL_LEDS_ON;

/* This is to create /dev/helloplus device nodes */

static char mydev_name[]="helloplus";  // This will appears in /proc/devices
static struct cdev  *hello_cdev;
static struct class *hello_class;
static dev_t   dev;

static void my_wq_function(struct delayed_work *work) {
	my_work_t *my_work = (my_work_t *)work;
	printk("my work.delay %d\n", my_work->delay);

	int led;
	switch(my_work->led) {
		case 0:
			led = ALL_LEDS_ON;
			break;
		case 1:
			led = CAP_LED_ON;
			break;
		case 2:
			led = SCROLL_LED_ON;
			break;
		case 3:
			led = NUM_LED_ON;
			break;
		default:
			led = ALL_LEDS_ON;
	}

	if(my_work->led_status == 0) {
		led = ALL_LEDS_OFF;
		my_work->led_status = 1;
	} else {
		my_work->led_status = 0;
	}

	(my_driver->ops->ioctl) (vc_cons[fg_console].d->port.tty, KDSETLED, led);
	queue_delayed_work(my_wq, work, my_work->delay);
	return;
}


static long hello_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
      switch(cmd){
      case  GET_DELAY:
       	if (!arg)  // Null pointer
	 		return -EINVAL; 
        printk("GET_DELAY was issued \n");
       	if (copy_to_user((long *)arg, &secs2hello, sizeof(long)))
       		return -EFAULT;
        printk(KERN_INFO " value is:%ld\n",secs2hello);
        return 0;
      case SET_DELAY:
        if (!arg)
	 		return -EINVAL;
        printk(KERN_INFO "set to:  %ld\n", secs2hello);
        if (copy_from_user(&secs2hello, (long *) arg,  sizeof (long)))
            return -EFAULT;
        printk(KERN_INFO "set to:  %ld\n", secs2hello);
	 	/*
	  	*  New timer is requested by user. This version guarantees that 
	 	 *  that the timer function itself is not running when it returns
	  	*  This will avoid any race condition in smp environment
	  	*/
	  	if(delayed_work_pending(&(work->my_work))) {
			cancel_delayed_work_sync(&(work->my_work));
	  	}
	  	work->delay = secs2hello*HZ;
	  	queue_delayed_work(my_wq, (struct delayed_work *)work, work->delay);
		return 0;
	  case SET_LED:
		if(!arg)
			return -EINVAL;
		printk("SET_LED was issued\n");
		if(copy_from_user(&led2hello, (long *)arg,  sizeof(long)))
			return -EFAULT;
		printk(KERN_INFO "set to: %ld\n", led2hello);
		if(delayed_work_pending(&(work->my_work))) {
			cancel_delayed_work_sync(&(work->my_work));
		}
		work->led = led2hello;
		queue_delayed_work(my_wq, (struct delayed_work *)work, work->delay);
	  default:  	// unknown command 
	    return -ENOTTY;
	}
     return -ENOTTY;
}

static int hello_open(struct inode *inode, struct file *file)
{
	printk("helloplus: open\n");
	return 0;
}


static int hello_release(struct inode *inode, struct file *file)
{
	printk("helloplus: release\n");
	return 0;
}

static struct file_operations hello_fops = {
	open: 	 hello_open,
	release: hello_release,
	unlocked_ioctl:   hello_ioctl,
	owner:	 THIS_MODULE
};

static int __init helloplus_init(void)
{
	int result;
	int major;
	int ret;

	printk("In init module");


	/** 
 	  * Dynamically allocate Major Number.  
	  * If you always want the same major number then use MKDEV and 
	  * register_chrdev
	  * dev = MKDEV(DEV_MAJOR, DEV_MINOR);
	  * ret = register_chrdev(dev,HELLOPLUS,&hello_fops)
	  * unregister it on failure by: unregister_chrdev(DEV_MAJOR,HELLOPLUS);
	  */

    result = alloc_chrdev_region(&dev, 0, 1, mydev_name);
    if (result<0) 
		return result;

	printk("The device is registered by Major no: %d\n", dev);
	 major = MAJOR(dev);

	// Allocate a cdev structure 
    hello_cdev = cdev_alloc();
	
	// Attach hello fops methods with the cdev: hello_cdev->ops=&hello_fops 
	cdev_init (hello_cdev, &hello_fops);
    hello_cdev->owner = THIS_MODULE;

	// Connect the assigned major number to the cdev 
    result = cdev_add(hello_cdev, dev, 1);
    if (result<0){
	  printk("Error in registering the module\n");
      unregister_chrdev_region(dev, 1);
      return result;
    }

	printk(KERN_INFO "helloplus: %d\n",__LINE__);

    /*
	 * Create an entry (class/directory) in sysfs using:
	 * class_create() and device_create()
     * for udev to automatically create a device file when module is 
     * loaded and this init function is called.
     */

    hello_class = class_create(THIS_MODULE,mydev_name);
	if (IS_ERR(hello_class)) {
                printk(KERN_ERR "Error creating hello class.\n");
                result = PTR_ERR(hello_class);
                cdev_del(hello_cdev);
                unregister_chrdev_region(dev, 1);
                return -1;
    }

    device_create(hello_class,NULL,dev,NULL,"helloplus%d",0);

	printk(KERN_INFO "helloplus: %d\n",__LINE__);

	my_driver = vc_cons[fg_console].d->port.tty->driver;
	printk(KERN_INFO "kbleds: tty driver magic %x\n", my_driver->magic);

	secs2hello = INITIAL_SECS;
	led2hello = ALL_LEDS_ON;
	my_wq = create_workqueue("my_queue");
	if(my_wq) {
		/* Queue some work */
		work = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
		if(work) {
			INIT_DELAYED_WORK((struct delayed_work *)work, my_wq_function);
			work->delay = secs2hello;
			work->led = led2hello;
			work->led_status = 1;
			ret = queue_delayed_work(my_wq, (struct delayed_work *)work, secs2hello);
		}
	}

	printk(KERN_INFO "helloplus: %d\n",__LINE__);
	printk(KERN_INFO "helloplus: loading\n");

	return 0;
}

static void __exit helloplus_cleanup(void)
{
	printk(KERN_INFO "helloplus: unloading...\n");

	if(delayed_work_pending(&(work->my_work))) {
		cancel_delayed_work_sync(&(work->my_work));
		printk(KERN_ALERT "Cancelling delayed work...\n");
	}

	flush_workqueue(my_wq);
	kfree(work);
	destroy_workqueue(my_wq);

	cdev_del(hello_cdev);
	device_destroy(hello_class, dev);
	class_destroy(hello_class);

	unregister_chrdev_region(dev,1);

	(my_driver->ops->ioctl) (vc_cons[fg_console].d->port.tty, KDSETLED, ALL_LEDS_OFF);
	printk("In cleanup module\n");
}

module_init(helloplus_init);
module_exit(helloplus_cleanup);
