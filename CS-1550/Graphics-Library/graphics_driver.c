/**
 * @author Zachary M. Mattis
 * COE 1550
 * Graphics Driver
 * May 31, 2018
 *
 * This program is used to test the functionality
 * of the Graphics Library (graphics.h) by calling
 * each of the implemented functions.
 */

/* Header Files */
#include "graphics.h"

/* Macros */
#define Y_RES 480
#define X_RES 1280

int main() {

  // variables
  color_t clr;
  void *buff;
  int i, j;

  clr = RGB(31, 0, 0);
  printf("clr: %d\n", clr);

  // init
  init_graphics();

  buff = new_offscreen_buffer();

  for (i=X_RES/4; i<X_RES/2; i++){
    for (j=100; j<150; j++){
      draw_pixel(buff, i, j, clr);
    }
  }
  blit(buff);
  sleep_ms(1000);

  draw_line(buff, 0, 100, X_RES/2, 200, RGB(0, 0, 31));
  draw_line(buff, 0, 100, X_RES/2, 50, RGB(0, 63, 0));
  draw_line(buff, X_RES/2, 100, 0, 150, RGB(15, 15, 15));
  draw_line(buff, 100, 100, 100, 150, RGB(31, 0, 31));
  blit(buff);
  sleep_ms(10000);

  clear_screen(buff);

  exit_graphics();

  return 0;
}