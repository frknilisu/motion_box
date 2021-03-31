#!/usr/bin/python3

import rospy
import sys
import time
import random
import struct

from std_msgs.msg import Int64

import RPi.GPIO as GPIO


class Encoder:

    def __init__(self, pins):
        self.clk, self.dt = pins
        self.setup()

        self.counter = 0
        self.clkLastState = GPIO.input(self.clk)

        self.pub_counter = rospy.Publisher(
            "/encoder_counter", Int64, queue_size=10)

    def setup(self):
        GPIO.setup(self.clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def update(self):
        clkState = GPIO.input(self.clk)
        dtState = GPIO.input(self.dt)
        if clkState != self.clkLastState:
            if dtState != clkState:
                self.counter += 1
            else:
                self.counter -= 1

            print(self.counter)

        self.clkLastState = clkState

        new_msg = Int64()
        new_msg.data = self.counter
        self.pub_counter.publish(new_msg)


if __name__ == "__main__":
    rospy.init_node('encoder', log_level=rospy.DEBUG)
    encoder = Encoder(pins=(17, 18))
    rospy.spin()
