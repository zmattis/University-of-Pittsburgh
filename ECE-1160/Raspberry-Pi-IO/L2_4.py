#! /usr/bin/python
    
# course:      ECE 1160
# laboratory:  2
# date:        10/04/18
# username:    zmm15
# name:        Zachary M. Mattis
# title:       Raspberry Pi SenseHat Oblique Lines
# description: SenseHat LED matrix manipulation w/ custom oblique lines

from sense_hat import SenseHat
import color
import time

SLEEP_TIME = 0.5
GRID_SIZE = 8
LINES = (GRID_SIZE * 2) - 1

sense_hat = SenseHat()


def switcher(evt):
  return {
    1: color.BLACK,
    2: color.RED,
    3: color.GREEN,
    4: color.BLUE,
    5: color.YELLOW,
    6: color.CYAN,
    7: color.MAGENTA,
    8: color.WHITE
  }[evt]
		

while True:
	for line in range(LINES):
		sense_hat.clear()
		for y in range(GRID_SIZE):
			for x in range(GRID_SIZE):
				if ( x+y == line ):
					sense_hat.set_pixel( x, y, switcher( abs(GRID_SIZE-line) ))
		time.sleep(SLEEP_TIME)