/* Sample code fragments and template for Assignment 1 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>         // struct file_operations
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>      // put_user
#include <linux/tty.h>        // tty
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/device.h>  // dynamic device node creation     

static char my_devname[]= "poll_dev"; // appears in /proc/devices

#define  N 23
static char rbuf[N+1];  // You can also use kmalloc() memory for your device 
static int  use_count;
static int  ir, iw;
static spinlock_t lock = SPIN_LOCK_UNLOCKED;

#define DEV_MAJOR	249 // for static  major and minor device node 
#define DEV_MINOR	5

static DECLARE_WAIT_QUEUE_HEAD(qin); // Wait queue head for read event 
static struct semaphore sema;

static int is_buffer_empty(void)
{
      if (ir == iw)
         return 1;

     return 0;
}

static unsigned int device_poll(struct file *filp, poll_table *wait)
{
        unsigned int mask=0;

        poll_wait(filp, &qin, wait);

        /* Serialize access */
        spin_lock(&lock);

        if(!is_buffer_empty()) 
        {
          printk ("%s - POLLIN EVENT:ir=%d|iw=%d\n", my_devname,ir,iw);
               mask |= POLLIN | POLLRDNORM;  /* fd is readable */
        }

	spin_unlock(&lock);

 return mask;
}


static ssize_t device_read(struct file *filp, 
                           char *buffer, size_t len, loff_t *offs)
{
  DECLARE_WAITQUEUE(wait, current); /* contains a pointer to a task struct */
  unsigned int i=0;

     if (is_buffer_empty()) /* no data to read */
	if (filp->f_flags & O_NONBLOCK) 
	 return -EAGAIN;

     add_wait_queue(&qin, &wait); /* Link our wait struct to the qin waitq */

     while(is_buffer_empty()){
        printk(" %s read - blocking \r\n", my_devname);
        set_current_state(TASK_INTERRUPTIBLE);
       if(!is_buffer_empty()) /* There is a data in device */
          break;  
       schedule();
       if(signal_pending(current)) {
         set_current_state(TASK_RUNNING);
         remove_wait_queue(&qin,&wait);
         return -ERESTARTSYS;
       } /* singal */
     } /* while */

  set_current_state(TASK_RUNNING);
  remove_wait_queue(&qin,&wait);

  if (down_interruptible(&sema))
        return -ERESTARTSYS;

   while(i<len){
    if(ir==iw) break;
    put_user(rbuf[ir++], buffer++);
    i++; 
    if(ir==N)  // end of device, reset the ir pointer to begining
      ir=0;
  }

  printk("\n %s read - %d bytes \r\n", my_devname, i);

   up(&sema);

// add code to wake up writers. Writers are not blocked in this version 

  return i;
}

static ssize_t device_write(struct file *filp, 
                            const char *buffer, size_t len, loff_t *offs)
{

 //add code to put a process to sleep if no room to write 

  unsigned int i=0;
  int ih;

   if (down_interruptible(&sema))  // serialize access to device 
      return -ERESTARTSYS;

  ih=ir-1; 
  if(ih<0) ih+=N;

  while(i<len){
    if(iw==ih) break;
    get_user(rbuf[iw++], buffer++);
    i++; 
    if(iw==N) iw=0;
  }

  printk(" %s write - %d bytes \r\n", my_devname, i);

  up(&sema);

  /* Wake up readers since device has atleast one byte of data*/
  if (i>0) wake_up_interruptible(&qin);

  return i;
}

static int device_open(struct inode *inode, struct file *filp)
{
  int use;

  spin_lock(&lock);
  if( use_count == 0 )
  {
    ir=iw=0;
  }
  use_count++; use=use_count;
  spin_unlock(&lock);

  printk(" %s open - count: %d \r\n", my_devname, use);

  return 0;
}

static int device_release(struct inode *inode, struct file *filp)
{
  int use;

  spin_lock(&lock);
  use = --use_count;
  spin_unlock(&lock);

  printk(" %s close - count %d \r\n", my_devname, use);

  return 0;
} 

//......................................................

static struct file_operations fops = {
  owner:   THIS_MODULE,
  read:    device_read,       
  write:   device_write,
  poll:    device_poll,
  open:    device_open,
  release: device_release,
};

static        dev_t   dev;
static struct cdev   *my_cdev;

unsigned int wait_major;
unsigned int wait_minor;

static int __init my_init(void)
{
  int res=0, major;

  sema_init(&sema,1); /* initialize usage count as 1, acts like mutex */
/**
  *  static device major and minor
  wait_major = DEV_MAJOR;
  wait_minor = DEV_MINOR;
  dev = MKDEV(wait_major, wait_minor);
  res = register_chrdev(dev,my_devname,&fops);

  printk("The device is registered by Major no: %d\n",res);
  if(res == -1)
  {
    printk("Error in registering the module\n");
    return -1;
  }
*/

/* Dynamic major number */
 res = alloc_chrdev_region(&dev, 0, 1, my_devname);
 if (res<0) { return res; }

  major = MAJOR(dev); /* device major number */

 /* allocate cdev structure and point to our device fops*/
  my_cdev = cdev_alloc();
  cdev_init(my_cdev, &fops);
  my_cdev->owner = THIS_MODULE;

/* connect major to cdev struct */
  res = cdev_add(my_cdev, dev, 1);
  if (res<0){
    unregister_chrdev_region(dev, 1);
    return res;
  }

  use_count = 0;  /* initialize use count */

  printk(" %s init - major: %d  \r\n", my_devname, major);

  return res;
}

static void __exit my_exit(void)
{
  cdev_del(my_cdev);
  unregister_chrdev_region(dev, 1);

 // For static device nodes, use unregister_chrdev(wait_major,my_devname);
 
  printk(" %s remove \r\n", my_devname);
}

module_init(my_init);
module_exit(my_exit);

MODULE_AUTHOR("Your Name");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Description");

