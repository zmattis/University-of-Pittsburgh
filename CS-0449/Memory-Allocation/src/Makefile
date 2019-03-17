# @author Zachary M. Mattis
# CS 0449
# Memory Management
# July 10, 2017
#
# This Makefile provides the compilation
# instructions for the custom Memory Allocation

CC = gcc
CFLAGS = -m32

default: malloc_driver

malloc_driver: mem_malloc.o malloc_driver.o
	$(CC) $(CFLAGS) -o malloc_driver mem_malloc.o malloc_driver.o

mem_malloc.o: mem_malloc.c mem_malloc.h
	$(CC) $(CFLAGS) -c mem_malloc.c

mallocdrv.o: malloc_driver.c mem_malloc.h
	$(CC) $(CFLAGS) -c malloc_driver.c

clean:
	rm -f malloc_driver *.o
