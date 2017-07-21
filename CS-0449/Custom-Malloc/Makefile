# Makefile for custom Malloc() implementation

CC = gcc
CFLAGS = -m32

default: mallocdrv

mallocdrv: mymalloc.o mallocdrv.o
	$(CC) $(CFLAGS) -o mallocdrv mymalloc.o mallocdrv.o

mymalloc.o: mymalloc.c mymalloc.h
	$(CC) $(CFLAGS) -c mymalloc.c

mallocdrv.o: mallocdrv.c mymalloc.h
	$(CC) $(CFLAGS) -c mallocdrv.c

clean:
	rm -f mallocdrv *.o
