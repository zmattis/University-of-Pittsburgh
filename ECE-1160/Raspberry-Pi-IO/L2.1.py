#!/usr/bin/python
    
# course:      ECE 1160
# laboratory:  2
# date:        10/04/18
# username:    zmm15
# name:        Zachary M. Mattis
# title:       Raspberry Pi SenseHat LED
# description: Dynamic and Continuous display over SenseHat LED Matrix

from sense_hat import SenseHat
import color
import time

SLEEP_TIME = 1
NUM_SCROLL = 3


class Arrow(object):
    left   = [Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE]
    center = [Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE]
    right  = [Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED, Color.WHITE,
              Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.WHITE, Color.RED]
    blank  = [Color.WHITE] * 64

    sense_hat = SenseHat()

    @staticmethod
    def scroll(duration):
        for i in duration:
            sense_hat.set_pixels(this.right)
            time.sleep(SLEEP_TIME)
            sense_hat.set_pixels(this.center)
            time.sleep(SLEEP_TIME)
            sense_hat.set_pixels(this.left)
            time.sleep(SLEEP_TIME)
            sense_hat.set_pixels(this.blank)
            time.sleep(SLEEP_TIME)


Arrow.scroll(NUM_SCROLL)
