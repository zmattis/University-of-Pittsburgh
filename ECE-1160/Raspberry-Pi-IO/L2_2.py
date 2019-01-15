#!/usr/bin/python
    
# course:      ECE 1160
# laboratory:  2
# date:        10/04/18
# username:    zmm15
# name:        Zachary M. Mattis
# title:       Raspberry Pi SenseHat Joystick
# description: SenseHat LED Matrix manipulation based on user SebseHat Joystick input

#from sense_hat import SenseHat
import color
import time

SCROLL_SPEED = 0.1

sense_hat = SenseHat()

def switcher(evt):
    return {
        'up': up
        'down': down
        'left': left
        'right': right
        'middle': middle
        }[evt]

# Display Temperature
def up():
    temp = sense_hat.get_temperature()
    sense_hat.show_messsage(temp, SCROLL_SPEED, Color.BLUE, Color.YELLOW)

#Display Pressure
def down():
    pressure = sense_hat.get_pressure()
    sense_hat.show_message(temp, SCROLL_SPEED, Color.YELLOW, Color.BLUE)

#Display Arrow
def left():

#
def right():
	
#action
def middle():

    
    

while True:
    for event in sense_hat.get_events()
        if (event.action == 'pressed')
            switcher(event.direction)
