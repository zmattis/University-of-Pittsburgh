# Syscalls

CS 1550 Assignment 2

## Description

The TrafficSim program models a common roadway occurrence where a lane is closed and a flagperson is directing traffic. By only allowing 1 side of traffic to pass through at any given time, this situation can accurately be represented via a binary semaphore where the road is the critical resource. The other half of this program is the actual system call implementations themselves within the Linux Kernel. These include a semaphore struct variable, as well as increment & decrement operations. This project implementation uses QEMU as a Linux Kernel emulator.

## Usage

#### Adding a New System Call

To add a new syscall to the Linux kernel, there are three main files that need to be modified:

  1. linux-2.6.23.1/kernel/**sys.c**

This file contains the actual implementation of the system calls.

  2. linux-2.6.23.1/arch/i386/kernel/**syscall_table.S**

This file declares the number that corresponds to the syscalls.

  3. linux-2.6.23.1/include/asm/**unistd.h**

This file exposes the syscall number to C programs which wish to use it.

#### Rebuilding the Kernel

To build any changes you made, from the linux-2.6.23.1/ directory, simply:

```bash
$ make ARCH=i386 bzImage
```

#### Copying the Files to QEMU

From QEMU, you will need to download two files from the new kernel that you just built. The kernel itself is a file named bzImage that lives in the directory linux-2.6.23.1/arch/i386/boot/ . There is also a supporting file called System.map in the linux-2.6.23.1/ directory that tells the system how to find the system calls.

For a remote machine, use scp to download the kernel to a home directory (/root/ if root):

```bash
$ scp USERNAME@DOMAIN:/PATH/linux-2.6.23.1/arch/i386/boot/bzImage .
$ scp USERNAME@DOMAIN:/PATH/linux-2.6.23.1/System.map .
```

#### Installing the Rebuilt Kernel in QEMU / Booting

As root (either by logging in or via su):

```bash
$ cp bzImage /boot/bzImage-devel
$ cp System.map /boot/System.map-devel
```

and respond ‘y’ to the prompts to overwrite. Please note that only the -devel files are replaced, so that other, original files allow for a clean version to boot QEMU in the occurrence of a boot failure.

You need to update the bootloader when the kernel changes. To do this (do it every time you install a new kernel if you like) as root type:

```bash
$ lilo
```

lilo stands for Linux Loader, and is responsible for the menu that allows you to choose which version of the kernel to boot into.

As root, you simply can use the reboot command to cause the system to restart.

```bash
$ reboot
```
When LILO starts (the red menu), make sure to use the arrow keys to select the linux(devel) option and hit enter.

#### Building _TrafficSim_

When building the trafficsim.c program on your remote machine, gcc will return an undefined symbol error due to the modification of the unistd.h file. In order to include the newly modified file, use the -I option:

```bash
$ gcc -m32 -o trafficsim -I /PATH/linux-2.6.23.1/include/ trafficsim.c
```

#### Execution

The trafficsim program cannot be run on your remote machine because its kernel does not have the new syscalls in it. To transfer the compiled program to the modified QEMU kernel, use scp in QEMU like previously demonstrated:

```bash
$ scp USERNAME@DOMAIN:/PATH/trafficsim .
```

To run the trafficsim program, simply execute the compiled binary:

```bash
$ ./trafficsim
```

## File Details

<dl>
  <dt>sys.c</dt>
  <dd>Implementation of the system calls</dd>
  <dt>syscall_table.S</dt>
  <dd>Declares the number that corresponds to the syscalls</dd>
  <dt>unistd.h</dt>
  <dd>Exposes the syscall number to C applications</dd>
  <dt>trafficsim.c</dt>
  <dd>Traffic Simulator utilizing semaphore and system calls from modified kernel</dd>
  <dt>linux-2.6.23.1.tar.bz2</dt>
  <dd>Linux Kernel v2.6.23.1</dd>
  <dt>qemu.zip</dt>
  <dd>Linux Kernel emulator</dd>
  <dt>syscalls_description.pdf</dt>
  <dd>PDF description from professor</dd>
</dl>

## Project Hierarchy

Drivers
  - trafficsim.c

Library
  - sys.c
  - syscall_table.S
  - unistd.h

Kernel
  - linux-2.6.23.1.tar.bz2
  - qemu.zip

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
