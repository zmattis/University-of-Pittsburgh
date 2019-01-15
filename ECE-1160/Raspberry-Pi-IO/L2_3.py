#! /usr/bin/python
    
# course:      ECE 1160
# laboratory:  2
# date:        10/04/18
# username:    zmm15
# name:        Zachary M. Mattis
# title:       Raspberry Pi SenseHat Joystick
# description: SenseHat LED Oblique Lines

from sense_hat import SenseHat
import color
import time

GRID_SIZE = 8
SLEEP_TIME = 0.1


sense_hat = SenseHat()

# Display the letter J
sense_hat.show_letter('Z')


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
  
  	  
def slide(): 
	for line in range(GRID_SIZE):
		sense_hat.clear()
		for x in range(GRID_SIZE):
			for y in range(GRID_SIZE):
				if (y == line):
					sense_hat.set_pixel( x, y, switcher(line) )
		time.sleep(SLEEP_TIME)
		

while True:
	acceleration = sense.get_accelerometer_raw()
	x = round(acceleration['x'], 0)
	y = round(acceleration['y'], 0)
	z = round(acceleration['z'], 0)

	
	print("x={0}, y={1}, z={2}".format(x, y, z))

    # Update the rotation of the display depending on which way up the Sense HAT is
	if x  == -1:
	  sense.set_rotation(180)
	elif y == 1:
	  sense.set_rotation(90)
	elif y == -1:
	  sense.set_rotation(270)
	else:
	  sense.set_rotation(0)
	  

		
		
