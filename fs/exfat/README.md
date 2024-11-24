## exFAT filesystem 
This is the exfat filesystem for support from the linux 4.1 kernel
to the latest kernel. 

## Installing as a stand-alone module

Install prerequisite package for Fedora, RHEL:
```
	yum install kernel-devel-$(uname -r)
```

Build step:
```
	make
	sudo make install
```

To load the driver manually, run this as root:
```
	modprobe exfat
```


## Installing as a part of the kernel

1. Let's take [linux] as the path to your kernel source dir.
```
	cd [linux]
	cp -ar exfat [linux]/fs/
```

2. edit [linux]/fs/Kconfig
```
	source "fs/fat/Kconfig"
	+source "fs/exfat/Kconfig"
	source "fs/ntfs/Kconfig"
```

3. edit [linux]/fs/Makefile
```
	obj-$(CONFIG_FAT_FS)          += fat/
	+obj-$(CONFIG_EXFAT_FS)       += exfat/
	obj-$(CONFIG_BFS_FS)          += bfs/
```
4. make menuconfig and set exfat
```
	File systems  --->
		DOS/FAT/NT Filesystems  --->
			<M> exFAT filesystem support
			(utf8) Default iocharset for exFAT
```

build your kernel
