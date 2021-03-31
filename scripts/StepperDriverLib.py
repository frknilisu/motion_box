import RPi.GPIO as GPIO
import time
import sys
import math

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

"""
mode_pins = (14, 15, 18)    # Microstep Resolution MS1-MS3 -> GPIO Pin
step_pin = 21                # Step -> GPIO Pin
direction_pin = 20           # Direction -> GPIO Pin
enable_pin = 
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

        print("StepperDriver initialized")

    def clearPins(self):
        GPIO.output(self.step_pin, GPIO.LOW)
        GPIO.output(self.direction_pin, GPIO.LOW)
        GPIO.output(self.enable_pin, GPIO.HIGH)
        GPIO.output(self.mode_pins, (GPIO.LOW, )*len(self.mode_pins))

    def enable(self, enable):
        GPIO.output(self.enable_pin, not enable)

    def step(self):
        GPIO.output(self.step_pin, GPIO.HIGH)
        #time.sleep(self.step_delay)
        GPIO.output(self.step_pin, GPIO.LOW)
        #time.sleep(self.step_delay)
        self.stepCount += 1
    
    def degToSteps(self, degree):
        return math.floor(degree / self.deg_per_step)
        """
        step_type
        microsteps =  {'Full': 1,
                        'Half': 2,
                        '1/4': 4,
                        '1/8': 8,
                        '1/16': 16,
                        '1/32': 32}
        return int(round((deg / 1.8) * microsteps[step_type]))
        """

    def steps2Deg(self, steps):
        return float(steps * self.deg_per_step)

    def set_resolution(self, step_type):
        """ method to calculate step resolution
        based on motor type and steptype"""
        if self.motor_type == "A4988":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (1, 1, 1)}
        elif self.motor_type == "DRV8825":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (0, 0, 1),
                          '1/32': (1, 0, 1)}
        else: 
            print("Error invalid motor_type: {}".format(self.motor_type))
            quit()
        
        # error check stepmode
        if step_type in resolution:
            pass
        else:
            print("Error invalid steptype: {}".format(step_type))
            quit()

        self.steps_per_rev = self.deg2Steps(360)
        GPIO.output(self.mode_pins, resolution[step_type])
        #resolution / (60 * rpm)
 
    def rotate(self, degree=90.0, clockwise=False, rpm=6, step_type="Full", verbose=False, init_delay=.05):
        """ rotate, moves stepper motor based on 6 inputs
         (1) degree, type=float, default=90.0, help=Number of degree sequence's to execute. Default is one revolution , 200 in Full mode.
         (2) clockwise, type=bool default=False, help="Turn stepper counterclockwise"
         (3) rpm, type=int, default=6, help=Speed in rpm (in seconds) between steps.
         (4) step_type, type=string, default=Full, help=Type of drive to step motor 5 options (Full, Half, 1/4, 1/8, 1/16)
         (5) verbose, type=bool, default=False, help="Write pin actions",
         (6) initdelay, type=float, default=1mS, help=Intial delay after GPIO pins initialized but before motor is moved.
        """

        try:
            # dict resolution
            self.set_resolution(step_type)

            time.sleep(init_delay)

            # Calculate time between steps in seconds
            step_delay = 60.0 / (self.steps_per_rev * rpm)

            # Convert degrees to steps
            numberOfSteps = self.deg2Steps(degree)

            print("step_delay: {}, # of steps: {}".format(step_delay, numberOfSteps))

            GPIO.output(self.direction_pin, clockwise)
            self.enable(True)

            for _ in range(0, numberOfSteps):
                self.step()
                time.sleep(step_delay)
                if verbose:
                    print("stepCount: {}, stepsToDeg: {:.2f}".format(self.stepCount, self.steps2Deg(self.stepCount)))

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")
        else:
            # print report status
            if verbose:
                print("\nRpiMotorLib, Motor Run finished, Details:.\n")
                print("Motor type = {}".format(self.motor_type))
                print("Clockwise = {}".format(clockwise))
                print("Step Type = {}".format(step_type))
                print("Number of steps = {}".format(numberOfSteps))
                print("Step Delay = {}".format(step_delay))
                print("Intial delay = {}".format(init_delay))
                print("Size of turn in degrees = {}"
                      .format(degree_calc(steps, steptype)))
        finally:
            self.clearPins()


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

    def deg2Steps(self, degree):
        #return int(round(deg/self.stepSize))
        return math.floor(degree / self.deg_per_step)

    def steps2Deg(self, steps):
        return float(steps * self.deg_per_step)

    def set_resolution(self, step_type):
        if step_type == "Half":
            self.steps_per_rev = 4096.0    # steps per revolution

        self.deg_per_step = 360.0 / self.steps_per_rev      # degree per step

    def rotate(self, degree=90.0, clockwise=False, rpm=3, step_type="Half", verbose=False, init_delay=.05):
        """ rotate, moves stepper motor based on 6 inputs
         (1) clockwise, type=bool default=False help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to step motor 5 options (Full, Half, 1/4, 1/8, 1/16)
         (3) steps, type=int, default=200, help=Number of steps sequence's to execute. Default is one revolution , 200 in Full mode.
         (4) stepdelay, type=float, default=0.05, help=Time to wait (in seconds) between steps.
         (5) verbose, type=bool  type=bool default=False help="Write pin actions",
         (6) initdelay, type=float, default=1mS, help= Intial delay after GPIO pins initialized but before motor is moved.
        """
        
        try:
            self.set_resolution(step_type)

            time.sleep(init_delay)

            # Calculate time between steps in seconds
            step_delay = 60.0 / (self.steps_per_rev * rpm)
            
            # Convert degrees to steps
            numberOfSteps = self.deg2Steps(degree)

            print("step_delay: {}, # of steps: {}".format(step_delay, numberOfSteps))

            if not clockwise:
                self.pins.reverse()

            for _ in range(0, numberOfSteps):
                self.step()
                time.sleep(step_delay)
                if verbose:
                    print("stepCount: {}, stepsToDeg: {:.2f}".format(self.stepCount, self.steps2Deg(self.stepCount)))
            
            print("stepCount: {}, stepsToDeg: {:.2f}".format(self.stepCount, self.steps2Deg(self.stepCount)))
            
            if not clockwise:
                self.pins.reverse()

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")
        else:
            # print report status
            if verbose:
                print("\nRpiMotorLib, Motor Run finished, Details:.\n")
                print("Motor type = {}".format(self.motor_type))
                print("Clockwise = {}".format(clockwise))
                print("Step Type = {}".format(step_type))
                print("Number of steps = {}".format(numberOfSteps))
                print("Step Delay = {}".format(step_delay))
                print("Intial delay = {}".format(init_delay))
                print("Size of turn in degrees = {}"
                      .format(degree_calc(steps, steptype)))
        finally:
            self.clearPins()
