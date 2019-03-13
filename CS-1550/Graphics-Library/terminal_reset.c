/**
 * @author Zachary M. Mattis
 * COE 1550
 * Terminal Reset
 * May 31, 2018
 *
 * This program resets keypress echoing to display typing to the
 * screen automatically and re-enables buffering of the keypresses.
 * It can be used after abnormal program termination of the Graphics
 * Library (graphics.h) to reset the terminal standards.
 */

/* Header Files */
#include <termios.h>

int main() {

  // variables
  struct termios currT, newT;

  // renable echoing and buffering keypresses
  ioctl(STDIN_FILENO, TCGETS, &currT);
  newT = currT;
  newT.c_lflag |= (ECHO | ICANON);	// enable echo & canonical modes
  ioctl(STDIN_FILENO, TCSETS, &newT);

  return 0;
}
