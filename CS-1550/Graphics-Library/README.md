# Graphics Library

CS 1550 Assignment 1

## Description

The Graphics Library is an API implemented in C that can be used by user-space application programmers to interact with and modify the video display via a frame buffer. This API supports setting a pixel to a particular color, drawing some basic shapes, and reading keypresses.

## Library

| Function Call                                                         | System Call(s) Used |
| --------------------------------------------------------------------- | ------------------- |
| void init_graphics()                                                  | open, ioctl, mmap   |
| void exit_graphics()                                                  | ioctl               |
| char getkey()                                                         | select, read        |
| void sleep_ms(long ms)                                                | nanosleep           |
| void clear_screen(void \*img)                                         |                     |
| void draw_pixel(void \*img, int x, int y,color_t color)               |                     |
| void draw_line(void \*img, int x1, int y1, int x2, int y2, color_t c) |                     |
| void \*new_offscreen_buffer()                                         | mmap                |
| void blit(void \*src)                                                 |                     |

## Usage

To compile the library (.c) files, execute the following command:

```bash
$ gcc -o library library.c
```

The Graphics Library includes two driver programs that utilize the functions of the library. To compile either source (.c) file, execute the following command(s):

```bash
$ gcc -o graphics_driver graphics_driver.c
$ gcc -o hilbert hilbert.c
```

To run either program, execute the following command(s):

```bash
$ ./graphics_driver
$ ./hilbert
```

## File Details

<dl>
  <dt>graphics.h</dt>
  <dd>Header file containing the function declarations and macros for use by C applications</dd>
  <dt>library.c</dt>
  <dd>C implementation for the Graphic Library functions, implemented using Linux system calls</dd>
  <dt>graphics_driver.c</dt>
  <dd>Basic C driver program using the Graphics Library function calls</dd>
  <dt>hilbert.c</dt>
  <dd>Advanced C driver program using the library to create a Hilbert curve</dd>
  <dt>terminal_reset.c</dt>
  <dd>C application to reset terminal keypad echoing and buffering</dd>
  <dt>qemu-arm.zip</dt>
  <dd>Linux Kernel emulator</dd>
  <dt>graphics_library_description.pdf</dt>
  <dd>PDF description from professor</dd>
</dl>

## Project Hierarchy

Drivers
  - graphics_driver.c
  - hilbert.c

Library
  - graphics.h
  - library.c

Utility
  - terminal_reset.c

Kernel
  - qemu-arm.zip

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
