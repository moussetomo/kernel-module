#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/proc_fs.h>

MODULE_AUTHOR("Masatomo Takai");
MODULE_LICENSE("GPL");

#define LEN (16*1024)
#define PROC_NAME "mmapper"
#define PROC_DIR_NAME "mmapper_dir"
#define NUMDEVS 2
#ifndef VM_RESERVED
#define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
#endif
char *dev_name_list[NUMDEVS] = {
	"kmalloc_dev",
	"vmalloc_dev",
};

dev_t firstdevno;

struct dev_struct {
	struct cdev cdev;
	dev_t devno;
};
static struct dev_struct mmapper_dev[NUMDEVS];

static char *vmalloc_ptr = NULL;
static char *kmalloc_ptr = NULL;
static char *kmalloc_area = NULL;
static struct class *mmapper_class;

static struct proc_dir_entry *proc_dir;
static struct proc_dir_entry *proc_mmapper;

static int mmapper_open(struct inode *inode, struct file *file) {
	int minor;
	minor = iminor(inode);
	printk("mmapper open with %d\n", minor);
	file->private_data = inode;
	return 0;
}

static int mmapper_release(struct inode *inode, struct file *file) {
	printk("mmapper release\n");
	return 0;
}

static int mmap_kmalloc(struct file *file, struct vm_area_struct *vma) {
	int ret;
	unsigned long length;
	length = vma->vm_end - vma->vm_start;

	// Restrict to size of device memory
	if(length > LEN * PAGE_SIZE)
			return -EIO;

	/* remap_pfn_range function arguments:
	 * vma: vm_area_struct has passed to the mmap method
	 * vma->vm_start: start of mapping user address space
	 * Page frame number of first page that you can get by:
	 * 	virt_to_phys((void *)kmalloc_area) >> PAGE_SHIFT
	 * size: length of mapping in bytes which is simply vm_end - vm_start
	 * vma->vm_page_prot protection bits receivevd from the application
	 */
	vma->vm_flags |= VM_RESERVED;
	ret = remap_pfn_range(vma, vma->vm_start, virt_to_phys((void*)((unsigned long)kmalloc_area)) >> PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if(ret != 0) {
		return -EAGAIN;
	}

	return 0;
}

static int mmap_vmem(struct file *file, struct vm_area_struct *vma) {
	int ret;
	long length = vma->vm_end - vma->vm_start;
	unsigned long start = vma->vm_start;
	char *vmalloc_area_ptr = (char *)vmalloc_ptr;
	unsigned long pfn;

	/* Restrict it to size of device memory */
	if(length > LEN * PAGE_SIZE)
			return -EIO;

	/* Considering vmalloc pages are not contiguous in physical memory
	 * You need to loop over all pages and call remap_pfn_range
	 * for each page individually. Also, use
	 * vmalloc_to_pfn(vmalloc_area_prt)
	 * instead to get the page frame number of each virtual pages
	 */
	vma->vm_flags |= VM_RESERVED;
	while(length > 0) {
		pfn = vmalloc_to_pfn(vmalloc_area_ptr);
		printk("vmalloc_area_ptr: 0x%p \n", vmalloc_area_ptr);

		if((ret = remap_pfn_range(vma, start, pfn, PAGE_SIZE, vma->vm_page_prot)) < 0) {
			return ret;
		}
		start += PAGE_SIZE;
		vmalloc_area_ptr += PAGE_SIZE;
		length -= PAGE_SIZE;
	}
	return 0;
}

static int mmapper_mmap(struct file *file, struct vm_area_struct *vma) {
	struct inode *inode;
	int minor;

	printk("mmapper mmap\n");
	inode = file->private_data;
	minor = iminor(inode);
	printk("minor is %d\n", minor);

	if(minor) {
		return mmap_vmem(file, vma);
	} else {
		return mmap_kmalloc(file, vma);
	}
}

static ssize_t proc_read(struct file *file, char *buf, size_t len, loff_t *ppos) {
	int n = 0;

	if(*ppos) n = 0;
	else 
		n = sprintf(buf, "/dev/kmalloc_dev has %s\n/dev/vmalloc_dev has %s\n", kmalloc_area, vmalloc_ptr);
	
	if(n)
		*ppos = len;

	return n;
}

static struct file_operations mmapper_fops = {
	open: mmapper_open,
	release: mmapper_release,
	mmap: mmapper_mmap,
};

static struct file_operations proc_fops = {
	.owner = THIS_MODULE,
	.read = proc_read,
};

static int __init mmapper_init(void) {
	int minor;
	int result;
	int major;
	struct dev_struct *thisDev;

	unsigned long virt_addr;

	// allocate major number dynamically
	result = alloc_chrdev_region(&firstdevno, 0, NUMDEVS, "mmapper");
	if(result < 0) return result;

	major = MAJOR(firstdevno);

	mmapper_class = class_create(THIS_MODULE, "mmapper");
	if(IS_ERR(mmapper_class)) {
		printk(KERN_ERR "Error creating device class.\n");
		return  PTR_ERR(mmapper_class);
	}
	for(minor = 0; minor < ARRAY_SIZE(mmapper_dev); minor++) {
		thisDev = &(mmapper_dev[minor]);
		thisDev->devno = MKDEV(major, minor);
		printk(KERN_INFO "device created with major: %d and minor: %d", major, minor);
		// attach file operations to cdev
		cdev_init(&(thisDev->cdev), &mmapper_fops);
		thisDev->cdev.owner = THIS_MODULE;
		thisDev->cdev.ops = &mmapper_fops;
		// connect assigned major number to cdev
		result = cdev_add(&(thisDev->cdev), thisDev->devno, NUMDEVS);
		if(result) {
			printk(KERN_ALERT "Error (%d) adding %s(%d)\n", result, "dev", minor);
			return result;
		}

		device_create(mmapper_class, NULL, thisDev->devno, NULL, dev_name_list[minor]);
	}

	/* Allocate memory with vmalloc. It is already page aligned */
	vmalloc_ptr = (char*) vmalloc(LEN);
	if(!vmalloc_ptr) {
		printk("vmalloc failed\n");
		return -ENOMEM;
	}
	printk("vmalloc_ptr at 0x%p \n", vmalloc_ptr);

	for(virt_addr = (unsigned long) vmalloc_ptr; virt_addr < (unsigned long)vmalloc_ptr + LEN; virt_addr += PAGE_SIZE) {
		SetPageReserved(vmalloc_to_page((unsigned long *)virt_addr));
	}

	printk("vmalloc_ptr: 0x%p\n", vmalloc_ptr);
	printk("vmalloc_ptr: 0x%p \t physical Address 0x%lx\n", vmalloc_ptr, (long unsigned int)virt_to_phys((void*)(vmalloc_ptr)));
	
	/* Initialize memory with "abcdefghijklmnopqrstuvwxyz" to distinguish between kmalloc and vmalloc initialized mamory */
	memcpy(vmalloc_ptr, "abcdefghijklmnopqrstuvwxyz", 27);

	/* kmalloc() returns memory in bytes instead of PAGE_SIZE
	 * mmap memory should be PAGE_SIZE and aligned on a PAGE boundary
	 */
	kmalloc_ptr = kmalloc(LEN + (2 * PAGE_SIZE), GFP_KERNEL);
	if(!kmalloc_ptr) {
		printk("kmalloc failed\n");
		return -ENOMEM;
	}
	printk("kmalloc_ptr at 0x%p \n", kmalloc_ptr);

	/* This is the same as:
	 * (int *)((((unsigned long)kmalloc_ptr) + ((1<<12) - 1)) & 0xFFFF0000);
	 * where: PAGE_SIZE is defined as 1UL << PAGE_SHIFT.
	 * That is 4k on x86. 0xFFFF0000 is a PAGE_MASK to make out the upper
	 * bits in the page. This will align it at 4k page boundary that menas
	 * kmalloc start address is now page aligned
	 */
	kmalloc_area = (char *)(((unsigned long)kmalloc_ptr + PAGE_SIZE - 1) & PAGE_MASK);
	printk("kmalloc_area: 0x%p\n", kmalloc_area);

	/* reserve kmalloc memory as pages to make them remapable */
	for(virt_addr = (unsigned long)kmalloc_area; virt_addr < (unsigned long)kmalloc_area + LEN; virt_addr += PAGE_SIZE) {
		SetPageReserved(virt_to_page(virt_addr));
	}
	printk("kmalloc_area: 0x%p\n", kmalloc_area);
	printk("kmalloc_area: 0x%p \t physical Address 0x%lx\n", kmalloc_area, (long unsigned int)virt_to_phys((void *)(kmalloc_area)));

	/* Write code to init memory with ascii 0123456789. Where ascii equivalent of 0 is 48 and 9 is 58.
	 * This is read from mmap() by user level application
	 */
	memcpy(kmalloc_area, "0123456789", 11);

	proc_dir = proc_mkdir(PROC_DIR_NAME, 0);
	proc_mmapper = proc_create(PROC_NAME, 0777, proc_dir, &proc_fops);

	printk("module loaded\n");
	return 0;
}

static void __exit mmapper_exit(void) {
	int minor;
	struct dev_struct *thisDev;
	unsigned long virt_addr;

	for(virt_addr = (unsigned long)kmalloc_area; virt_addr < (unsigned long)kmalloc_area + LEN; virt_addr += PAGE_SIZE) {
		ClearPageReserved(virt_to_page(virt_addr));
	}
	kfree(kmalloc_ptr);

	for(virt_addr = (unsigned long)vmalloc_ptr; virt_addr < (unsigned long)vmalloc_ptr + LEN; virt_addr += PAGE_SIZE) {
		ClearPageReserved(vmalloc_to_page((unsigned long*)virt_addr));
	}
	vfree(vmalloc_ptr);

	for(minor = 0; minor < ARRAY_SIZE(mmapper_dev); minor++) {
		thisDev = &(mmapper_dev[minor]);
		cdev_del(&(thisDev->cdev));
		device_destroy(mmapper_class, thisDev->devno);
	}
	class_destroy(mmapper_class);
	
	if(proc_mmapper) {
		remove_proc_entry(PROC_NAME, proc_dir);
	}

	if(proc_dir) {
		remove_proc_entry(PROC_DIR_NAME, 0);
	}
	

	unregister_chrdev_region(firstdevno, NUMDEVS);
	printk("module unloaded\n");
}

module_init(mmapper_init);
module_exit(mmapper_exit);
