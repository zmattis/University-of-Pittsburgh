# Pi Linux Device Driver

CS 0449 Assignment 4

## Description

The pi device driver is a linux driver that creates a new device that returns the digits of pi, via /dev/pi. A testing program is included that utilizes the device driver to return pi.

## Usage

To compile the testing source (.c) file on your machine, execute the following command:

```bash
$ gcc -m32 -o pi_digits pi_digits.c
```

To view instructions for lading the driver into the linux kernel, see pi_linux_device_driver.pdf. Once the driver has been loaded into the linux kernel and the pi program has been compiled, test the /dev/pi device with the command below. Replace \<START\>, \<END\> with integers of the bounds of the digits of pi you wish to read:

```bash
$ ./pi_digits <START> <END>
```


## File Details

<dl>
  <dt>pi.h</dt>
  <dd>C header file containing function implementation of pi digits</dd>
  <dt>pi_driver.c</dt>
  <dd>Software interface of pi device (provides file operations for device)</dd>
  <dt>pi_digits.c</dt>
  <dd>Program that outputs the requested digits of pi via /dev/pi device</dd>
  <dt>Makefile</dt>
  <dd>Compilation file used to produce modules of pi_driver</dd>
  <dt>qemu.zip</dt>
  <dd>Linux Kernel emulator</dd>
  <dt>pi_linux_device_driver.pdf</dt>
  <dd>PDF description from professor</dd>
  <dt>linux_device_driver_intro.pdf</dt>
  <dd>Linux Device Driver reference material via O'Reilly Linux DevCenter</dd>
</dl>

## Project Hierarchy

Program Driver:
  - pi_digits.c

Linux Device Driver Library:
  - pi.h
  - pi_driver.c

Utility
  - Makefile

Linux Emulator:
  - qemu.zip

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
