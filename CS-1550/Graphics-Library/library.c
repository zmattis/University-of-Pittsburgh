/**
 * @author Zachary M. Mattis
 * COE 1550
 * Graphics Library
 * May 31, 2018
 *
 * This C file is the implementation of a Graphics Library (graphics.h)
 * using Linux system calls. It can set pixels to different colors,
 * draw basic shapes, and read keypresses.
 */


/* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include "graphics.h"

/* Macros */
#define FRAMEBUFFER "/dev/fb0"
#define NS_OFFSET 1000000

/* Function Prototypes */
int sign(int);
int abs_val(int);

/* Gloabls */
int fb_fd;
size_t g_yres, g_xres, g_resolution;
void *g_framebuffer;
void *g_doublebuffer;
struct timeval g_timeout;

void init_graphics() {
	//variables
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	struct termios currT, newT;

	//open framebuffer
	printf("DEBUG: Opening framebuffer\n");
	fb_fd = open(FRAMEBUFFER, O_RDWR);
	if (fb_fd == -1){
		perror("");
		exit(EXIT_FAILURE);
	}
	printf("DEBUG: Framebuffer opened\n");

	//get screen resolution
	printf("DEBUG: Getting screen resolution\n");
	if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0){
		perror("");
		exit(EXIT_FAILURE);
	}
	if (ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0){
		perror("");
		exit(EXIT_FAILURE);
	}
	printf("DEBUG: Screen resolution opened\n");

	//set globals
	g_yres = vinfo.yres_virtual;
	g_xres = finfo.line_length;
	g_resolution = g_yres * g_xres;

	//map screen memory
	printf("DEBUG: Mapping framebuffer to memory\n");
	g_framebuffer = mmap(NULL, g_resolution, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
	printf("DEBUG: Framebuffer mapped\n");
	printf("screen_size: %d\n", g_yres);
	printf("bits_per_pixel: %d\n", g_xres);
	printf("g_resolution: %d\n", g_resolution);

	//clear screen
	printf("DEBUG: Clearing Screen\n");
	//printf("\033[2J");
	printf("DEBUG: Screen clear\n");

	//disable echoing and buffering keypresses
	printf("DEBUG: Configuring terminal\n");
	ioctl(STDIN_FILENO, TCGETS, &currT);
	newT = currT;
	newT.c_lflag &= ~(ECHO | ICANON);	//disable echo & canonical modes
	ioctl(STDIN_FILENO, TCSETS, &newT);
	printf("DEBUG: Terminal configured\n");

	//set timeout =0 (non-blocking)
	g_timeout.tv_sec=0;
	g_timeout.tv_usec=0;

}

void exit_graphics () {
	//variables
	struct termios currT, newT;

	//close fd
	close (fb_fd);

	//unmap memory
	munmap(g_framebuffer, g_resolution);
	munmap(g_doublebuffer, g_resolution);

	//renable echoing and buffering keypresses
	ioctl(STDIN_FILENO, TCGETS, &currT);
	newT = currT;
	newT.c_lflag |= (ECHO | ICANON);	//enable echo & canonical modes
	ioctl(STDIN_FILENO, TCSETS, &newT);
}

char getkey() {
	fd_set fds;	//standard input file descriptor set
	int ready;
	char key;

	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);

	ready = select(1, &fds, NULL, NULL, &g_timeout);

	if (ready < 0){
		perror("");
		exit(EXIT_FAILURE);
	} else if (ready == 0) {
		return -1;
	} else {
		if (read(STDIN_FILENO, &key, 1) < 0 ){		// read 1 char from stdin
			perror("");
			exit(EXIT_FAILURE);
		}
		return key;
	}

}

void sleep_ms(long ms) {
	//variables
	struct timespec time;
	time.tv_sec = 0;
	time.tv_nsec = ms*NS_OFFSET;

	nanosleep(&time, NULL);
}

void *new_offscreen_buffer() {

	//map buffer
	g_doublebuffer = mmap(NULL, g_resolution, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (g_doublebuffer<0){
		perror("");
		exit(EXIT_FAILURE);
	}

	return g_doublebuffer;

}

void blit(void *src){
	//varaibles
	int i;
	char *buff, *display;

	display = (char *)g_framebuffer;
	buff = (char *)src;

	//mem cpy (byte by byte)
	for (i=0; i<g_resolution; i++){
		display[i] = buff[i];
	}

}

void clear_screen(void *img) {
	//variables
	int i;
	char *buff;

	buff = (char *)img;
	for (i=0; i<g_resolution; i++){
		buff[i]=0;
	}
}

void draw_pixel(void *img, int x, int y, color_t color){
	//varaibles
	color_t *buff;

	buff = (color_t *)img;
	*(buff + (y * g_xres) + x) = color;

}

void draw_line(void *img, int x1, int y1, int x2, int y2, color_t color){
	int x, y, dx, dy;
	int swap, temp, s1, s2, p, i;

	x=x1;
	y=y1;
	dx=abs(x2-x1);
	dy=abs(y2-y1);
	s1=sign(x2-x1);
	s2=sign(y2-y1);
	swap=0;
	if (dx==0) {	//vertical
		for(i=0;i<dy;i++){
			draw_pixel(img, x, y, color);
			y+=s2;
		}
	}
	if(dy>dx && dx!=0) {
		temp=dx;
		dx=dy;
		dy=temp;
		swap=1;
	}
	p=2*dy-dx;
	for(i=0;i<dx;i++) {
		draw_pixel(img, x, y, color);
		while(p>=0) {
			p=p-2*dx;
			if(swap){
				x+=s1;
			} else {
				y+=s2;
			}
		}

		p=p+2*dy;
		if(swap) {
			y+=s2;
		} else {
			x+=s1;
		}
	}

}

//Helper function: returns sign of x
int sign(int x) {
	if(x>0) {
		return 1;
	} else if(x<0) {
		return -1;
	} else {
		return 0;
	}
}

//Helper function: returns absolute value of x
int abs_val(int x){
	return (x < 0) ? -x : x;
}
