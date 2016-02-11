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

int main(void)
{
 int fd, ret,i;
 long Delay=1;

 if((fd = open("/dev/helloplus0", O_RDWR))<0) {
    perror("open error\n");
    exit(EXIT_FAILURE);
  }
  while ( i < 0){
  printf("\nHit return..");
  getchar();
  ret = ioctl(fd,  GET_DELAY, &Delay);
  printf("Delay is = %ld\n", Delay);
  Delay +=5;
  printf("\nSet Delay" );
  ret = ioctl( fd, SET_DELAY, &Delay);
  ret = ioctl( fd,  GET_DELAY, &Delay);
  printf("\nAfter ioctl call Delay = %ld\n",Delay);
  }

  close(fd);
  return 0;
}
