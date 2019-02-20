
#########################################################
# Makefile for Mandelbrot Set Application               #
#########################################################

CC      := gcc
CFLAGS  += -std=c99 -Wall -Wextra -Ilib -I. -O2
LDFLAGS += -lm

#########################################################
# Make All

all: mandelbrot

#########################################################
# Make Mandelbrot (Parallel)

mandelbrot:  mandelbrot.c
	$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

#########################################################
# Make Clean

clean:
	$(RM) mandelbrot *.ppm

.PHONY: clean

#########################################################



# END Makefile for Mandelbrot Set Application  - EWG SDG
