/* 
 *  kbleds.c - Blink keyboard leds until the module is unloaded.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/tty.h>		/* For fg_console, MAX_NR_CONSOLES */
#include <linux/kd.h>		/* For KDSETLED */
#include <linux/vt_kern.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/workqueue.h>

MODULE_DESCRIPTION("Example module illustrating the use of Keyboard LEDs.");
MODULE_AUTHOR("Masatomo Takai");
MODULE_LICENSE("GPL");

static struct workqueue_struct *my_wq;

typedef struct {
		struct delayed_work my_work;
		int delay;
} my_work_t;

my_work_t *work;
struct timer_list my_timer;
struct tty_driver *my_driver;
char kbledstatus = 0;

#define BLINK_DELAY   HZ
#define ALL_LEDS_ON   0x07
#define RESTORE_LEDS  0xFF

/*
 * Function my_timer_func blinks the keyboard LEDs periodically by invoking
 * command KDSETLED of ioctl() on the keyboard driver. To learn more on virtual 
 * terminal ioctl operations, please see file:
 *     /usr/src/linux/drivers/char/vt_ioctl.c, function vt_ioctl().
 *
 * The argument to KDSETLED is alternatively set to 7 (thus causing the led 
 * mode to be set to LED_SHOW_IOCTL, and all the leds are lit) and to 0xFF
 * (any value above 7 switches back the led mode to LED_SHOW_FLAGS, thus
 * the LEDs reflect the actual keyboard status).  To learn more on this, 
 * please see file:
 *     /usr/src/linux/drivers/char/keyboard.c, function setledstate().
 * 
 */

/*static void my_timer_func(unsigned long ptr)
{
	int *pstatus = (int *)ptr;

	if (*pstatus == ALL_LEDS_ON)
		*pstatus = RESTORE_LEDS;
	else
		*pstatus = ALL_LEDS_ON;
*/
	/** vc_cons is a struct type vc that contains a pointer to a 
	  * virtual console (d) of type vc_data	vc_tty is the tty port where 
	  * console is attached, fg_console is the current console 
	  * vt_ioctl(struct tty_struct *tty, struct file *file, 	
	  *                         unsigned int cmd, unsigned long arg)
	  */
/*
	(my_driver->ops->ioctl) (vc_cons[fg_console].d->port.tty, KDSETLED,
			    *pstatus);

	my_timer.expires = jiffies + BLINK_DELAY;
	add_timer(&my_timer);
}
*/
static void my_wq_function(struct delayed_work *work) {
	my_work_t *my_work = (my_work_t *)work;
	printk("my work.delay %d\n", my_work->delay);
	queue_delayed_work(my_wq, work, my_work->delay);
	return;
}

static int __init kbleds_init(void)
{
	int i, ret;

	printk(KERN_INFO "kbleds: loading\n");
	printk(KERN_INFO "kbleds: fgconsole is %x\n", fg_console);
	for (i = 0; i < MAX_NR_CONSOLES; i++) {
		if (!vc_cons[i].d)
			break;
		printk(KERN_INFO "poet_atkm: console[%i/%i] #%i, tty %lx\n", i,
		       MAX_NR_CONSOLES, vc_cons[i].d->vc_num,
		       (unsigned long)vc_cons[i].d->port.tty);
	}
	printk(KERN_INFO "kbleds: finished scanning consoles\n");

	my_driver = vc_cons[fg_console].d->port.tty->driver;
	printk(KERN_INFO "kbleds: tty driver magic %x\n", my_driver->magic);

	/*
	 * Set up the LED blink timer the first time
	 */
	/*init_timer(&my_timer);
	my_timer.function = my_timer_func;
	my_timer.data = (unsigned long)&kbledstatus;
	my_timer.expires = jiffies + BLINK_DELAY;
	add_timer(&my_timer);
	*/

	my_wq = create_workqueue("my_queue");
	if(my_wq) {
		/* Queue some work */
		work = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
		if(work) {
			INIT_DELAYED_WORK((struct delayed_work*)work, my_wq_function);
			work->delay = BLINK_DELAY;
			ret = queue_delayed_work(my_wq, (struct delayed_work *)work, BLINK_DELAY);
		}
	}
	return 0;
}

static void __exit kbleds_cleanup(void)
{
	printk(KERN_INFO "kbleds: unloading...\n");
	/*del_timer(&my_timer);
	(my_driver->ops->ioctl) (vc_cons[fg_console].d->port.tty, KDSETLED,
			    RESTORE_LEDS);
	*/
	if(delayed_work_pending(&(work->my_work))) {
		cancel_delayed_work_sync(&(work->my_work));
		printk(KERN_ALERT "Cancelling delayed work...\n");
	}
	printk("flushing\n");
	flush_workqueue(my_wq);
	printk("freeing\n");
	kfree(work);
	destroy_workqueue(my_wq);
}

module_init(kbleds_init);
module_exit(kbleds_cleanup);

