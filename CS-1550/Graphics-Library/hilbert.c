/*
This file draws a hilbert space filling curve. Initially it starts with
a simple U shape, but typing a + will increase the amount of space it has
to fill and will result in a fun curve. Eventually the lines it draws will
be length 1, and the whole 256x256 region will appear to be solid red.
*/

#include "graphics.h"

int direction = 0;
int curr_x = 0;
int curr_y = 0;

void turn_left(int degrees)
{
  direction = (direction + degrees + 360) % 360;
}

void go_forward(void *img, int distance)
{
  int new_x = curr_x;
  int new_y = curr_y;

  if (direction == 0)
    new_x += distance;
  else if (direction == 90)
    new_y += distance;
  else if (direction == 180)
    new_x -= distance;
  else if (direction == 270)
    new_y -= distance;

  draw_line(img, curr_x, curr_y, new_x, new_y, RGB(31, 0, 0));
  curr_x = new_x;
  curr_y = new_y;
}

void hilbert_recurse(void *img, int n, int parity, int dist)
{
  if (n == 0)
    return;

  turn_left(parity * 90);

  hilbert_recurse(img, n - 1, -parity, dist);
  go_forward(img, dist);
  turn_left(-parity * 90);

  hilbert_recurse(img, n - 1, +parity, dist);
  go_forward(img, dist);

  hilbert_recurse(img, n - 1, +parity, dist);
  turn_left(-parity * 90);
  go_forward(img, dist);

  hilbert_recurse(img, n - 1, -parity, dist);
  turn_left(parity * 90);
}

void hilbert(void *img, int n, int parity)
{
  hilbert_recurse(img, n, parity, 479 / (1 << n));
}

int main(int argc, char **argv)
{
  int i;

  init_graphics();

  //Construct an offscreen buffer to draw to
  void *buf = new_offscreen_buffer();

  char key;
  int n = 1;

  //Draw the simple U shape
  hilbert(buf, n, +1);
  blit(buf);

  do {
    curr_x = 0;
    curr_y = 0;

    key = getkey();
    if (key == 'q')
      break;
    //Make it more interesting
    else if (key == '+') {
      n++;
      clear_screen(buf);
      hilbert(buf, n, +1);
      blit(buf);
    }
    sleep_ms(200);
  }
  while (1);

  exit_graphics();
  return 0;

}
