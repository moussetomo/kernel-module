#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#define BUFSIZE 2*1024

int main()
{
        int i, fd_vmalloc, fd_kmalloc, len, wlen ;
        char * mptr_kmalloc, *mptr_vmalloc;
        size_t size = BUFSIZE;
        char buffer[BUFSIZE];

		fd_kmalloc = open("/dev/kmalloc_dev", O_RDWR | O_SYNC);
        fd_vmalloc = open("/dev/vmalloc_dev", O_RDWR | O_SYNC);
        if( fd_kmalloc == -1 || fd_vmalloc == -1) {
                printf("open error...\n");
                return -1;
        }
	
	/** 
	  * Requesting mmaping  at offset 0 on device memory, last argument. 
	  * This is used by driver to map device memory. First argument is the 
	  * virtual memory location in user address space where application 
	  * wants to setup the mmaping. Normally, it is set to 0  to allow 
	  * kernel to pick the best location in user address space. Please 
	  * review man pages: mmap(2)
	  */

		mptr_kmalloc = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_kmalloc, 0);
        mptr_vmalloc = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_vmalloc, 0);
        if(mptr_kmalloc == MAP_FAILED || mptr_vmalloc == MAP_FAILED) {
                printf("mmap() failed\n");
                return -1;
        }

        /**
	  * Now mmap memory region can be access as user memory. No syscall 
          * overhead! 
	  */

	/* read from mmap memory */
        printf("kmalloc mptr is %p\n", mptr_kmalloc);
        memset(buffer, 0, size);       		/* Clear the buffer */
        memcpy(buffer, mptr_kmalloc, size-1);  		/* Reading from kernel */
        printf("kmalloc mmap:  '%s'\n", buffer);
        printf("kmalloc mptr is %p\n", mptr_kmalloc);
		
		printf("vmalloc mptr is %p\n", mptr_vmalloc);
        memset(buffer, 0, size);       		/* Clear the buffer */
        memcpy(buffer, mptr_vmalloc, size-1);  		/* Reading from kernel */
        printf("vmalloc mmap:  '%s'\n", buffer);
        printf("vmalloc mptr is %p\n", mptr_vmalloc);


        /* write to mmap memory */
        memcpy(mptr_kmalloc, "MY KMALLOC STRING!", 19);
        memset(buffer, 0, size);
        memcpy(buffer, mptr_kmalloc, size-1);
        printf("kmalloc mmap:  '%s'\n", buffer);
		
		memcpy(mptr_vmalloc, "MY VMALLOC STRING!", 19);
        memset(buffer, 0, size);
        memcpy(buffer, mptr_vmalloc, size-1);
        printf("vmalloc mmap:  '%s'\n", buffer);


        munmap(mptr_kmalloc, size);
		munmap(mptr_vmalloc, size);
        close(fd_kmalloc);
		close(fd_vmalloc);
}
