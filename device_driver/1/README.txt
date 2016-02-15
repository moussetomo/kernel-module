Driver is developed under kernel 3.10.0-327.el7.x86_64.
It uses udev to create device file, so no need to run script.

Below is the output of test_pollRW.c

fd is writable

 WROTE 22 byte|size of buffer 62

Needs to write addtional 40 bytes
fd is readable

READ 22 byte: abcdefghijklmnopqrstuv

Needs to read addtional 40 bytes
fd is writable

 WROTE 22 byte|size of buffer 40

Needs to write addtional 18 bytes
fd is readable

READ 22 byte: wxyz0123456789ABCDEFGH

Needs to read addtional 18 bytes
fd is writable

 WROTE 18 byte|size of buffer 18

 All data is written
fd is readable

READ 18 byte: IJKLMNOPQRSTUVWXYZ

ALL DATA IS READ. Exiting ...
