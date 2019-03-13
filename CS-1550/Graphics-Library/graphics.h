/**
 * @author Zachary M. Mattis
 * COE 1550
 * Graphics Library
 * May 31, 2018
 *
 * This header file defines the user-facing
 * functions for interacting with the terminal
 * frame buffer via system calls.
 */

#ifndef __LIBRARY_H
#define __LIBRARY_H

#include <stdint.h>

#define RGB(r,g,b) \
    ((((r) & 31) << 11 ) | (((g) & 63) << 5 ) | ((b) & 31))
typedef int16_t color_t;

void init_graphics();
void exit_graphics();
char getkey();
void sleep_ms(long ms);
void clear_screen(void *img);
void draw_pixel(void *img, int x, int y, color_t color);
void draw_line(void *img, int x1, int y1, int x2, int y2, color_t c);
void *new_offscreen_buffer();
void blit(void *src);

#endif /* __LIBRARY_H */