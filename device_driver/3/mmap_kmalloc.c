/**
  *  This is not a complete program. Provided as an aid for 
  *  developing  mmap driver method 
  */

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


static char *kmalloc_area = NULL;
static char *kmalloc_ptr = NULL;

#define LEN (16*1024) 

unsigned long virt_addr;

static int mmap_kmalloc(struct file * filp, struct vm_area_struct * vma) {
        int ret;
        unsigned long length;
        length = vma->vm_end - vma->vm_start;

	// Restrict to size of device memory

        if (length > LEN * PAGE_SIZE)
           return -EIO;

	/**
          * remap_pfn_range function arguments:
          * vma: vm_area_struct has passed to the mmap method
          * vma->vm_start: start of mapping user address space
          * Page frame number of first page that you can get by:
          *   virt_to_phys((void *)kmalloc_area) >> PAGE_SHIFT
          * size: length of mapping in bytes which is simply vm_end - vm_start
          * vma->>vm_page_prot: protection bits received from the application
          */

        vma->vm_flags |= VM_RESERVED;
        ret = remap_pfn_range(
	       vma, 
	       vma->vm_start,
               virt_to_phys((void*)((unsigned long)kmalloc_area)) >> PAGE_SHIFT,
               vma->vm_end-vma->vm_start,
	       vma->vm_page_prot 
             );
        if(ret != 0) {
                return -EAGAIN;
        }
        return 0;
}


static int __init mmap_kmalloc_init_module (void) {
        int i;
        int ret;

   //Do required char driver initialization, see helloplus.c as an example 

   /**
     * kmalloc() returns memory in bytes instead of PAGE_SIZE
     * mmap memory should be PAGE_SIZE and aligned on a PAGE boundary.
     */

        kmalloc_ptr = kmalloc(LEN + (2 * PAGE_SIZE), GFP_KERNEL);
        if (!kmalloc_ptr) {
                printk("kmalloc failed\n");
                return -ENOMEM;
        }
        printk("kmalloc_ptr at 0x%p \n", kmalloc_ptr);

    /**
      * This is the same as: 
      * (int *)((((unsigned long)kmalloc_ptr) + ((1<<12) - 1)) & 0xFFFF0000);
      * where: PAGE_SIZE is defined as 1UL <<PAGE_SHIFT. 
      * That is 4k on x86. 0xFFFF0000 is a PAGE_MASK to mask out the upper 
      * bits in the page. This will align it at 4k page boundary that means 
      * kmalloc start address is now page aligned.
      */

        kmalloc_area = (char *)(((unsigned long)kmalloc_ptr + PAGE_SIZE -1) & PAGE_MASK);

        printk("kmalloc_area: 0x%p\n", kmalloc_area);

	/* reserve kmalloc memory as pages to make them remapable */
        for (virt_addr=(unsigned long)kmalloc_area; virt_addr < (unsigned long)kmalloc_area + LEN;
                virt_addr+=PAGE_SIZE) {
                        SetPageReserved(virt_to_page(virt_addr));
        }
        printk("kmalloc_area: 0x%p\n" , kmalloc_area);
        printk("kmalloc_area :0x%p \t physical Address 0x%lx)\n", kmalloc_area,
                         virt_to_phys((void *)(kmalloc_area)));

        /**
	  *  Write code to init memory with ascii 0123456789. Where ascii 
	  *  equivalent of 0 is 48  and 9 is 58. This is read from mmap() by 
	  *  user level application
	  */
	  
        return 0;
}

// close and cleanup module

static void __exit mmap_kmalloc__cleanup_module (void) {
        printk("cleaning up module\n");

        for (virt_addr=(unsigned long)kmalloc_area; virt_addr < (unsigned long)kmalloc_area + LEN;
                virt_addr+=PAGE_SIZE) {
                        // clear all pages
                        ClearPageReserved(virt_to_page(virt_addr));
        }
        kfree(kmalloc_ptr);

	// Also all required clean up for character drivers
}
