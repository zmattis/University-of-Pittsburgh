# Linux Device Driver

CS 0449 Assignment 4

## Description

The pi device driver is a linux driver that creates a new device that returns the digits of pi.

## Usage

Note: To use QEMU to emulate Linux, simply download qemu.zip, extract it, and run the qemu-win.bat file (Windows only).

To build the kernel object on your machine, use the Makefile, executing the following commmand:

* make

Note: Within Makefile, set the path KDIR to the directory of the kernel <br />
Note: To build 32-bit kernel using 64-bit machine, add flag ARCH=i386 (make ARCH=i386)

To load the driver into the kernel, execute the following commmand in Linux: 

* insmod pi_driver.ko

Note: The file pi_driver.ko needs to be present in your current directory ( /~ )

To create the pi device, find the device id, executing the following commands in Linux:

* cd /sys/class/misc/pi
* cat dev

This should output two numbers as:    MAJOR:MINOR

To create the new character device, execute the following commands in Linux:

* cd /dev/
* mknod pi c MAJOR MINOR

Note: Replace MAJOR, MINOR with the numbers found above. <br />
Note: These numbers are used to link drivers to their devices

To compile the source (.c) file on your machine, execute the following command:

* gcc -o pi_digits pi_digits.c

Note: To build as 32-bit on 64-bit machine, add flag -m32 (gcc -m32 -o pi_digits pi_digits.c)

Once the driver has been loaded into the linux kernel and the pi program has been compiled, execute the following command in Linux:

* ./pi_digits START END

Replace START, END with integers of the bounds of the digits of pi you wish to read.

## File Details

* pi_digits.c -- program the ouputs the requested digits of pi
* pi_driver.c -- software interface of pi device (provides file operations for device)
* pi.h -- header file containing function implementation of pi digits
* Makefile -- file used to produce modules of pi_driver
* qemu.zip -- compressed file containingLinux emulator
* a4description.pdf -- project description from professor
 
## Project Hierarchy

Program Driver:

* pi_digits.c -- pi_driver.c pi.h

Linux Device Driver:

* pi_driver.c -- pi.h

Linux Emulator:

* qemu.zip

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
