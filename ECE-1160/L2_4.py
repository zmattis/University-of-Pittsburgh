#!/usr/bin/python
    
# course:      ECE 1160
# laboratory:  2
# date:        10/04/18
# username:    zmm15
# name:        Zachary M. Mattis
# title:       Raspberry Pi SenseHat Joystick
# description: SenseHat Oblique Lines

from sense_hat import SenseHat
import color
import time

SLEEP_TIME = 1
GRID_SIZE = 8


sense_hat = SenseHat()
while true:
	for ii in range(15)
		sense_hat.clear()
		for jj in range(GRID_SIZE)
			for kk in range(GRID_SIZE)
				if (jj + kk == ii) {
					sense_hat.set_pixel(jj, kk, Color.CYAN)
				}
			
		time.sleep(SLEEP_TIME)		
	