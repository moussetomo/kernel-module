# proc entry before mmap
/dev/kmalloc_dev has 0123456789
/dev/vmalloc_dev has abcdefghijklmnopqrstuvwxyz

# mmap
kmalloc mptr is 0x7f25cf488000
kmalloc mmap:  '0123456789'
kmalloc mptr is 0x7f25cf488000
vmalloc mptr is 0x7f25cf487000
vmalloc mmap:  'abcdefghijklmnopqrstuvwxyz'
vmalloc mptr is 0x7f25cf487000
kmalloc mmap:  'MY KMALLOC STRING!'
vmalloc mmap:  'MY VMALLOC STRING!'

# proc entry after mmap
/dev/kmalloc_dev has MY KMALLOC STRING!
/dev/vmalloc_dev has MY VMALLOC STRING!
