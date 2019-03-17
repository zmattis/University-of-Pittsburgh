# Memory Allocation

CS 0449 Assignment 3

## Description

The Memory Allocation source provides a custom malloc() implementation that utilizes the *brk* heap pointer and a next-fit algorithm to manually manipulate the heap. This library features both memory allocation and freeing functions, as well as a driver program to demonstrates its behavior.

## Library

| Functions                          | Descriptions                                               |
| ---------------------------------- | ---------------------------------------------------------- |
| void *mem_nextfit_malloc(int size) | allocates memory using the next-fit algorithm              |
| void my_free(void *ptr)            | deallocates a pointer that was originally allocated memory |

## Usage

To utilize the custom function implementations, simply include the header file in your source:

```c
#include "mem_malloc.h"
```

To test the library and compile source (*.c) files, use the given Makefile:

```bash
$ make
```

To execute the driver program, run the compiled binary:

```bash
$ ./malloc_driver
```

## File Details

<dl>
  <dt>mem_malloc.h</dt>
  <dd>Header file containing the function declarations for use by C applications</dd>
  <dt>mem_malloc.c</dt>
  <dd>C implementation for the allocation  and free functions</dd>
  <dt>malloc_driver.c</dt>
  <dd>C driver program using the custom memory allocation library function calls</dd>
  <dt>Makefile</dt>
  <dd>Compilation instructions for the driver program with the library</dd>
  <dt>memory_allocation_description.pdf</dt>
  <dd>PDF description from professor</dd>
</dl>

## Project Hierarchy

Drivers
  - malloc_driver.c

Library
  - mem_malloc.h
  - mem_malloc.c

Utility
  - Makefile

## License

Everything in this repo is distributed under the MIT License.

See LICENSE for more information.
