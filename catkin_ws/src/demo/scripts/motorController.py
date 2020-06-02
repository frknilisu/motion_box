#!/usr/bin/python3

"""
Gear reduction  1/64
Step angle  
    Half step mode (recommended): 0.0879°
    Full step mode: 0.176°
Steps per revolution    
    Half step mode: 4096
    Full step mode: 2048
"""

import sys
import time
import random
import struct
import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import Int64
from demo.msg import CmdStepMsg

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)


class StepperMotor:

    Seq = [(1, 0, 0, 1),
           (1, 0, 0, 0),
           (1, 1, 0, 0),
           (0, 1, 0, 0),
           (0, 1, 1, 0),
           (0, 0, 1, 0),
           (0, 0, 1, 1),
           (0, 0, 0, 1)]

    revsteps = 4096                # steps per revolution
    stepSize = 360 / revsteps      # degree per step

    def deg2Steps(self, deg):
        return (int)(deg/self.stepSize)

    def steps2Deg(self, steps):
        return (float)(steps * self.stepSize)

    # void stop()
    # void currentState(float * position)
    # void run()

    def __init__(self, pins):
        assert len(pins) == 4, "4 pins must be specified"
        self.pins = pins
        self.setup()

        self.stepCount = 0
        self.seq_head = 0
        self.step_delay = 0.1

        self.pub_pose = rospy.Publisher(
            "/current_position", Int64, queue_size=10)
        self.sub_cmd_step = rospy.Subscriber(
            "/cmd_step", CmdStepMsg, self.callback_cmd_step)

    def callback_cmd_step(self, msg):
        speed = msg.speed
        degree = msg.degree
        direction = msg.direction
        self.move(speed, degree, direction)

        new_msg = Int64()
        new_msg.data = self.stepCount
        self.pub_pose.publish(new_msg)

    def setup(self):
        GPIO.setup(self.pins, GPIO.OUT)
        GPIO.output(self.pins, (False, )*len(self.pins))

    def step(self):
        """Take a step in current direction"""
        if self.direction:
            self.setPins(self.Seq[self.seq_head])
            self.seq_head += 1
        else:
            self.setPins(self.Seq[self.seq_head])
            self.seq_head -= 1

        time.sleep(self.step_delay)

        self.stepCount += 1
        self._next_seq()

    def _next_seq(self):
        if(self.seq_head >= len(self.Seq)):
            self.seq_head = 0
        elif(self.seq_head < 0):
            self.seq_head = len(self.Seq)-1

    def setPins(self, seq):
        """Set the pins with given sequence"""
        GPIO.output(self.pins, seq)

    def move(self, speed, degree, direction):
        self.direction = direction   # ccw: false, cw: true
        self.step_delay = 60.0 / (self.revsteps * speed)
        numberOfSteps = self.deg2Steps(degree)
        for _ in range(0, numberOfSteps):
            self.step()


if __name__ == "__main__":
    rospy.init_node('motorController')
    # Initialise the motor, specify the GPIO pins as a list
    motor = StepperMotor(pins=(7, 11, 13, 15))
    rospy.spin()
