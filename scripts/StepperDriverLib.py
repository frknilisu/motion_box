import RPi.GPIO as GPIO
import time
import sys
import math

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

import logging
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
logger = logging.getLogger(__name__)

"""
mode_pins = (14, 15, 18)    # Microstep Resolution MS1-MS3 -> GPIO Pin
step_pin = 21                # Step -> GPIO Pin
direction_pin = 20           # Direction -> GPIO Pin
enable_pin = 16
motor = A4988_Nema(step_pin, direction_pin, enable_pin, mode_pins, "DRV8825")
"""
class A4988_Nema(object):
    
    """ Class to control a Nema bi-polar stepper motor with a A4988 also tested with DRV8825"""
    def __init__(self, step_pin, direction_pin, enable_pin, mode_pins, motor_type="A4988"):
        """ class init method 5 inputs
        (1) step_pin type=int , help=GPIO pin connected to STEP of IC
        (2) direction_pin type=int , help=GPIO pin connected to DIR pin of IC
        (3) enable_pin type=int , help=GPIO pin connected to EN pin of IC
        (4) mode_pins type=tuple of 3 ints, help=GPIO pins connected to Microstep Resolution pins MS1-MS3 of IC
        (5) motor_type type=string, help=Type of motor two options: A4988 or DRV8825
        """
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        self.motor_type = motor_type

        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)

        self.clearPins()

        self.enable(False)

        self.stepCount = 0
        self.step_delay = 0.1

        logger.info("StepperDriver initialized")

    def clearPins(self):
        GPIO.output(self.step_pin, GPIO.LOW)
        GPIO.output(self.direction_pin, GPIO.LOW)
        GPIO.output(self.enable_pin, GPIO.HIGH)
        GPIO.output(self.mode_pins, (GPIO.LOW, )*len(self.mode_pins))

    def step(self):
        GPIO.output(self.step_pin, GPIO.HIGH)
        #time.sleep(self.step_delay)
        GPIO.output(self.step_pin, GPIO.LOW)
        #time.sleep(self.step_delay)
        self.stepCount += 1

    def move(self, steps):
        try:
            self.enable(True)
            for _ in range(0, steps):
                self.step()
                time.sleep(self.step_delay)
                logger.debug("stepCount: {}, stepsToDeg: {:.2f}".format(self.stepCount, self.stepsToDegree(self.stepCount)))
        except KeyboardInterrupt:
            logger.info("User Keyboard Interrupt : StepperDriverLib:")
        except Exception as motor_error:
            logger.error(sys.exc_info()[0])
            logger.error(motor_error)
            logger.error("StepperDriverLib  : Unexpected error:")
        else:
            logger.debug("\nStepperDriverLib, Motor Run finished, Details:.\n")
            logger.debug("Motor type = {}".format(self.motor_type))
            #logger.debug("Clockwise = {}".format(clockwise))
            #logger.debug("Step Type = {}".format(step_type))
            logger.debug("Number of steps = {}".format(steps))
            logger.debug("Step Delay = {}".format(self.step_delay))
            logger.debug("Size of turn in degrees = {}".format(self.stepsToDegree(steps)))
        finally:
            self.clearPins()
            self.enable(False)

    def rotate(self, degree=90.0):
        """ rotate, moves stepper motor based on 6 inputs
        Number of degree sequence's to execute.
        """
        numberOfSteps = self.degreeToSteps(degree)
        logger.info("step_delay: {}, # of steps: {}".format(self.step_delay, numberOfSteps))
        self.move(numberOfSteps)

    def degreeToSteps(self, degree):
        return math.floor((degree / 1.8) * self.microstep)

    def stepsToDegree(self, steps):
        return float(steps * self.deg_per_step)

    def enable(self, enable):
        GPIO.output(self.enable_pin, not enable)

    def setMicrostep(self, microstep="Full"):
        """ 
        Type of drive to step motor 5 options (Full, Half, 1/4, 1/8, 1/16)
        """
        microsteps =  { 'Full': 1,
                        'Half': 2,
                        '1/4': 4,
                        '1/8': 8,
                        '1/16': 16,
                        '1/32': 32 }
        self.microstep = microsteps[microstep]

        if self.motor_type == "A4988":
            resolution = { 'Full': (0, 0, 0),
                           'Half': (1, 0, 0),
                           '1/4': (0, 1, 0),
                           '1/8': (1, 1, 0),
                           '1/16': (1, 1, 1) }
        elif self.motor_type == "DRV8825":
            resolution = { 'Full': (0, 0, 0),
                           'Half': (1, 0, 0),
                           '1/4': (0, 1, 0),
                           '1/8': (1, 1, 0),
                           '1/16': (0, 0, 1),
                           '1/32': (1, 0, 1) }
        else: 
            logger.error("Error invalid motor_type: {}".format(self.motor_type))

        GPIO.output(self.mode_pins, resolution[microstep])
        
        self.steps_per_rev = self.degreeToSteps(360.0)
        self.deg_per_step = 360.0 / self.steps_per_rev      # degree per step

    def setRPM(self, rpm=3):
        """
        Speed in rpm (in seconds) between steps.
        """
        self.rpm = rpm
        # Calculate time between steps in seconds
        self.step_delay = 60.0 / (self.steps_per_rev * self.rpm)


    def setDirection(self, clockwise):
        GPIO.output(self.direction_pin, clockwise)


"""
motor = StepperMotor(pins=[11, 15, 16, 18])
"""
class ULN2003A_BYJ(object):

    Seq = [(1, 0, 0, 1),
           (1, 0, 0, 0),
           (1, 1, 0, 0),
           (0, 1, 0, 0),
           (0, 1, 1, 0),
           (0, 0, 1, 0),
           (0, 0, 1, 1),
           (0, 0, 0, 1)]    

    def __init__(self, pins):
        self.pins = pins

        GPIO.setup(self.pins, GPIO.OUT)
        self.clearPins()

        self.stepCount = 0

        self.steps_per_rev = 4096.0
        self.deg_per_step = 360.0 / self.steps_per_rev

        print("Stepper initialized")

    def setPins(self, seq):
        """Set the pins with given sequence"""
        GPIO.output(self.pins, seq)
    
    def clearPins(self):
        self.setPins((GPIO.LOW, )*len(self.pins))

    def step(self):
        """Take a step in current direction"""
        self.setPins(self.Seq[self.stepCount % len(self.Seq)])
        self.stepCount += 1

    def move(self, steps):
        try:
            for _ in range(0, steps):
                self.step()
                time.sleep(self.step_delay)
                logger.debug("stepCount: {}, stepsToDeg: {:.2f}".format(self.stepCount, self.stepsToDegree(self.stepCount)))
        except KeyboardInterrupt:
            logger.info("User Keyboard Interrupt : StepperDriverLib:")
        except Exception as motor_error:
            logger.error(sys.exc_info()[0])
            logger.error(motor_error)
            logger.error("StepperDriverLib  : Unexpected error:")
        else:
            logger.debug("\nStepperDriverLib, Motor Run finished, Details:.\n")
            logger.debug("Motor type = {}".format(self.motor_type))
            #logger.debug("Clockwise = {}".format(clockwise))
            #logger.debug("Step Type = {}".format(step_type))
            logger.debug("Number of steps = {}".format(steps))
            logger.debug("Step Delay = {}".format(self.step_delay))
            logger.debug("Size of turn in degrees = {}".format(self.stepsToDegree(steps)))
        finally:
            self.clearPins()

    def rotate(self, degree=90.0):
        """ 
        rotate, moves stepper motor based on 6 inputs
        """
        numberOfSteps = self.degreeToSteps(degree)
        logger.info("step_delay: {}, # of steps: {}".format(self.step_delay, numberOfSteps))

        if not self.clockwise:
            self.pins.reverse()

        self.move(numberOfSteps)

        if not self.clockwise:
            self.pins.reverse()

    def degreeToSteps(self, degree):
        return math.floor((degree / 1.8) * self.microstep)

    def stepsToDegree(self, steps):
        return float(steps * self.deg_per_step)

    def enable(self, enable):
        pass

    def setMicrostep(self, microstep="Full"):
        """ rotate
        Type of drive to step motor 5 options (Full, Half, 1/4, 1/8, 1/16)
        """
        microsteps =  { 'Full': 1,
                        'Half': 2,
                        '1/4': 4,
                        '1/8': 8,
                        '1/16': 16,
                        '1/32': 32 }
        self.microstep = microsteps[microstep]

        self.steps_per_rev = self.degreeToSteps(360.0)
        self.deg_per_step = 360.0 / self.steps_per_rev      # degree per step

    def setRPM(self, rpm=3):
        """
        Speed in rpm (in seconds) between steps.
        """
        self.rpm = rpm
        # Calculate time between steps in seconds
        self.step_delay = 60.0 / (self.steps_per_rev * self.rpm)

    def setDirection(self, clockwise):
        self.clockwise =  clockwise
    