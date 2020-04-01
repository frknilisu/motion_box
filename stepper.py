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
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

class StepperMotor:

    Seq = [[1,0,0,1],
           [1,0,0,0],
           [1,1,0,0],
           [0,1,0,0],
           [0,1,1,0],
           [0,0,1,0],
           [0,0,1,1],
           [0,0,0,1]]

    def __init__(self, pins):
        assert len(pins) == 4, "4 pins must be specified"
        self.pins = pins
        self.direction = True   # ccw: false, cw: true
        self.stepCount = 0
        self.delay = 0.005

    def setup(self):
        GPIO.setup(self.pins, GPIO.OUT)
        GPIO.output(self.pins, (False, )*len(self.pins))

    def step(self):
        """Take a step in current direction"""
        print('{} {}'.format(self.stepCount, self.direction))
        if self.direction:
            for i in range(len(self.Seq)):
                self.setPins(self.Seq[i])
                time.sleep(self.delay)
        else:
            for i in reversed(range(len(self.Seq))):
                self.setPins(self.Seq[i])
                time.sleep(self.delay)

        self.stepCount += 1

    def setPins(self, seq):
        """Set the pins with given sequence"""
        assert len(seq) == len(self.pins), "length of sequence must be same with number of pins"
        for idx, pin in enumerate(self.pins):
            GPIO.output(pin, seq[idx])

    def turnToAngle(self, angle):

        def degree2Step(degree):
            return (int)(degree / 360 * (4096/8))

        for i in range(0, degree2Step(angle)):
            self.step()


if __name__ == "__main__":
    #Initialise the motor, specify the GPIO pins as a list
    motor = StepperMotor((7,11,13,15))
    motor.setup()

    if len(sys.argv) >= 3:
        angleToGo = float(sys.argv[1])
        waitTime = float(sys.argv[2])/float(1000)
        direction = str(sys.argv[3]) in ['true', '1', 'yes', 'True']

        motor.delay = waitTime
        motor.direction = direction
        motor.turnToAngle(angleToGo)
    else:
        """Decision-Maker game"""
        direction = bool(random.randint(0, 2))
        steps = random.randint(100, 512)
        print("# of steps will be taken: {} in {} direction".format(steps, "CW" if direction else "CCW"))
        motor.direction = direction
        for i in range(0, steps):
            motor.step()

    GPIO.cleanup()