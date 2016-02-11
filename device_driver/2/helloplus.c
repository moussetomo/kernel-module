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


MODULE_DESCRIPTION("Example driver illustrating the use of Timers and IOCTL.");
MODULE_AUTHOR("Your Name Here");
MODULE_LICENSE("GPL");

struct timer_list my_timer;
struct tty_driver *my_driver;
char helloplustatus = 0;

#define MAGIC 'z'
#define GET_DELAY      _IOR(MAGIC, 1, int *)
#define SET_DELAY      _IOW(MAGIC, 2, int *)

/**
Don't need it if you want major number assigned to your module dynamically 
#define DEV_MAJOR       250
#define DEV_MINOR       5
#define HELLOPLUS "helloplus"
*/

#define INITIAL_SECS 	HZ/2
static unsigned long secs2hello = INITIAL_SECS;


/* This is to create /dev/helloplus device nodes */

static char mydev_name[]="helloplus";  // This will appears in /proc/devices
static struct cdev  *hello_cdev;
static struct class *hello_class;
static dev_t   dev;


// Timer function 
static void my_timer_func(unsigned long ptr)
{
	my_timer.expires = jiffies + secs2hello;
	printk("helloplus: timer %lu %lu\n",jiffies,my_timer.expires);
	add_timer(&my_timer);
}


static int hello_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
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
	del_timer_sync(&my_timer);
        my_timer.function = my_timer_func;
        my_timer.data = (unsigned long)&helloplustatus;
        my_timer.expires = jiffies + secs2hello;
        add_timer(&my_timer);
	return 0;
	break;
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
	ioctl:   hello_ioctl,
	owner:	 THIS_MODULE
};

static int __init helloplus_init(void)
{
	int result;
	int major;

	printk("In init module");

	secs2hello = INITIAL_SECS;

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

	// set up  timer the first time

	init_timer(&my_timer);
	my_timer.function = my_timer_func;
	my_timer.data = (unsigned long)&helloplustatus;
	my_timer.expires = jiffies + secs2hello;  /* Set it to expire every second */
	add_timer(&my_timer);


	printk(KERN_INFO "helloplus: %d\n",__LINE__);
	printk(KERN_INFO "helloplus: loading\n");


	return 0;
}

static void __exit helloplus_cleanup(void)
{
	printk(KERN_INFO "helloplus: unloading...\n");
	del_timer(&my_timer);

	cdev_del(hello_cdev);
	device_destroy(hello_class, dev);
	class_destroy(hello_class);

	unregister_chrdev_region(dev,1);

	printk("In cleanup module\n");

}

module_init(helloplus_init);
module_exit(helloplus_cleanup);
