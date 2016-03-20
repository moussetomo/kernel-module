#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <linux/ioctl.h>

#define MAGIC 'z'
#define GET_DELAY      _IOR(MAGIC, 1, int *)
#define SET_DELAY      _IOW(MAGIC, 2, int *)
#define SET_LED		   _IOW(MAGIC, 3, int *)

#define ALL_LEDS 0
#define CAP_LED 1
#define SCROLL_LED 2
#define NUM_LED 3

int main(void)
{
 int fd, ret, i;
 long Delay=1;
 long led = CAP_LED;

 if((fd = open("/dev/helloplus0", O_RDWR))<0) {
    perror("open error\n");
    exit(EXIT_FAILURE);
  }
  for (i = 0; i < 10; i++){
  	printf("\nEnter interval(s)..");
  	scanf("%ld", &Delay);
	printf("\nEnter LED(0:ALL_LED, 1:CAP, 2:SCROLL, 3:NUM)..");
	scanf("%ld", &led);
  	printf("\nSet Delay" );
  	ret = ioctl( fd, SET_DELAY, &Delay);
  	ret = ioctl( fd,  GET_DELAY, &Delay);
  	printf("\nAfter ioctl call Delay = %ld\n",Delay);
	ret = ioctl( fd, SET_LED, &led);
  }

  close(fd);
  return 0;
}
