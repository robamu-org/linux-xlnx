# Xiphos Linux Kernel

This is a fork of the [Xilinx Linux kernel](https://github.com/Xilinx/linux-xlnx) for EIVE.
You can also find instructions to configure and build the kernel in the Q7S user manual starting
at page 20.

Make sure that all prerequisites are installed and setup before running the commands here.

To configure the kernel you can run the following command

```sh
make menuconfig
```

To configure the kernal based on an old `.config` file, use

```sh
make oldconfig
```

To build a new kernel image run the following command.

```sh
make LDFLAGS="" Image
```

Command to create an image with the U-Boot header directly

```sh
make ARCH=arm UIMAGE_LOADADDR=0x8000 uImage
```

# Prerequisites

You need to have the Yocto SDK installed, which only works on a Linux host. Building 
was tested on Ubuntu 21.04, 16.04 and 18.04.

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

# Enabling CAN Driver

In .config file make sure the following variables are set to 'y'.

```
CONFIG_CAN=y
CONFIG_CAN_DEV=y
CONFIG_CAN_XILINXCAN=y
```

Other notable changes and settings

```
# CONFIG_SERIAL_UARTLITE_CONSOLE is not set
CONFIG_SERIAL_UARTLITE_NR_UARTS=5

CONFIG_CAN_RAW=y
CONFIG_CAN_BCM=y
CONFIG_CAN_GW=y
CONFIG_CAN_SLCAN=y
# CONFIG_CAN_LEDS is not set
```

# Troubleshooting

* After changing the kernel configuration run 'make clean', otherwise booting the kernel may fail.
