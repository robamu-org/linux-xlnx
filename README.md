# Xiphos Linux Kernel

This is a fork of the [Xilinx Linux kernel](https://github.com/Xilinx/linux-xlnx)for EIVE.
You can also find instructions to configure and build the kernel in the Q7S user manual starting
at page 20.

Make sure that all prerequisites are installed and setup before running the commands here.

To configure the kernel you can run the following command

```sh
make menuconfig
```

To build a new kernel image run the following command.

```sh
make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage
```

# Prerequisites

You need to have the Yocto SDK installed, which only works on a Linux host. 
See Q7S user manual page 19 for more information. You can install the necessary files 
[from here](https://trac2.xiphos.ca/manual/attachment/wiki/Q7RevB/UserManual/xsc-release-1542-xsc-q7-2.2.4-4ba6f44b.tar).

After you have downloaded and extracted the files, you can run the following command
to set up the build environment

```sh
. /opt/xiphos/sdk/ark/environment-setup-<ARCH>-xiphos-<EABI>
```

It might be necessary to install the `ncurses` lib on some hosts to use `menuconfig`

```sh
sudo apt-get install libncurses-dev
```

# Documentation

This file was moved to Documentation/admin-guide/README.rst

Please notice that there are several guides for kernel developers and users.
These guides can be rendered in a number of formats, like HTML and PDF.

In order to build the documentation, use ``make htmldocs`` or
``make pdfdocs``.

There are various text files in the Documentation/ subdirectory,
several of them using the Restructured Text markup notation.
See Documentation/00-INDEX for a list of what is contained in each file.

Please read the Documentation/process/changes.rst file, as it contains the
requirements for building and running the kernel, and information about
the problems which may result by upgrading your kernel.

# Enabling CAN Driver

In .config file make sure the following variables are set to 'y'.

CONFIG_CAN=y  
CONFIG_CAN_DEV=y  
CONFIG_CAN_XILINXCAN=y  

# Troubleshooting

* After changing the kernel configuration run 'make clean', otherwise booting the kernel may fail.
