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

import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
#from demo.msg import CmdStepMsg


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

    def setup(self):
        pass

    def step(self, speed, direction):
        """Take a step in current direction"""
        self.direction = direction   # ccw: false, cw: true
        self.step_delay = 60.0 / (self.revsteps * speed)

        if self.direction:
            self.setPins(self.Seq[self.seq_head])
            self.seq_head += 1
        else:
            self.setPins(self.Seq[self.seq_head])
            self.seq_head -= 1

        if(self.seq_head >= len(self.Seq)):
            self.seq_head = 0
        elif(self.seq_head < 0):
            self.seq_head = len(self.Seq)-1

        self.stepCount += 1
        time.sleep(self.step_delay)

    def setPins(self, seq):
        """Set the pins with given sequence"""
        pass


def callback_keyboard_teleop(msg):
    speed = msg.linear.x
    direction = bool(msg.angular.z)
    print("speed: {}, direction: {}".format(speed, direction))
    motor.step(speed, direction)

    new_msg = Int64()
    new_msg.data = motor.stepCount
    pub_pose.publish(new_msg)


if __name__ == "__main__":
    rospy.init_node('motorController')
    # Initialise the motor, specify the GPIO pins as a list
    motor = StepperMotor(pins=(7, 11, 13, 15))
    pub_pose = rospy.Publisher("/current_position", Int64, queue_size=10)
    sub_teleop = rospy.Subscriber(
        "/cmd_step", Twist, callback_keyboard_teleop)
    rospy.spin()
