import RPi.GPIO as GPIO
from time import sleep
import math

"""
GPIO_pins = (14, 15, 18)    # Microstep Resolution MS1-MS3 -> GPIO Pin
directionPin = 20           # Direction -> GPIO Pin
stepPin = 21                # Step -> GPIO Pin
motor = A4988Nema(directionPin, stepPin, GPIO_pins, "DRV8825")
"""
class A4988Nema(object):

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    """ Class to control a Nema bi-polar stepper motor with a A4988 also tested with DRV8825"""
    def __init__(self, direction_pin, step_pin, mode_pins, motor_type="A4988"):
        """ class init method 3 inputs
        (1) direction type=int , help=GPIO pin connected to DIR pin of IC
        (2) step_pin type=int , help=GPIO pin connected to STEP of IC
        (3) mode_pins type=tuple of 3 ints, help=GPIO pins connected to
        Microstep Resolution pins MS1-MS3 of IC
        (4) motor_type type=string, help=TYpe of motor two options: A4988 or DRV8825
        """
        self.motor_type = motor_type

        self.direction_pin = direction_pin
        self.step_pin = step_pin
        self.mode_pins = mode_pins

        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)

        self.clearPins()

        self.stepCount = 0

        print("Stepper initialized")

    def clearPins(self):
        GPIO.output(self.direction_pin, GPIO.LOW)
        GPIO.output(self.step_pin, GPIO.LOW)
        GPIO.output(self.mode_pins, (GPIO.LOW, )*len(self.mode_pins))

    def step(self):
        GPIO.output(self.step_pin, GPIO.HIGH)
        GPIO.output(self.step_pin, GPIO.LOW)
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
        """ motor_go,  moves stepper motor based on 6 inputs
         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to
         step motor 5 options
            (Full, Half, 1/4, 1/8, 1/16)
         (3) steps, type=int, default=200, help=Number of steps sequence's
         to execute. Default is one revolution , 200 in Full mode.
         (4) stepdelay, type=float, default=0.05, help=Time to wait
         (in seconds) between steps.
         (5) verbose, type=bool  type=bool default=False
         help="Write pin actions",
         (6) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.
        """

        try:
            # dict resolution
            self.set_resolution(step_type)

            sleep(init_delay)

            # Calculate time between steps in seconds
            step_delay = 60.0 / (self.steps_per_rev * rpm)
            
            # Convert degrees to steps
            numberOfSteps = self.deg2Steps(degree)

            print("step_delay: {}, # of steps: {}".format(step_delay, numberOfSteps))

            if not clockwise:
                self.pins.reverse()

            for _ in range(0, numberOfSteps):
                self.step()
                sleep(step_delay)
                if verbose:
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



"""
class DRV8825:
    def __init__(self, stepPin, dirPin, enablePin, modePins):
        self.stepPin = stepPin
        self.dirPin = dirPin
        self.enablePin = enablePin
        self.modePins = modePins

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.stepPin, GPIO.OUT)
        GPIO.setup(self.dirPin, GPIO.OUT)
        GPIO.setup(self.enablePin, GPIO.OUT)
        GPIO.setup(self.modePins, GPIO.OUT)

        GPIO.output(self.modePins, (GPIO.HIGH, GPIO.LOW, GPIO.HIGH)) # 1/32 resolution stepping

        self.deg_per_step = 5.625 / 64  # for half-step drive (mode 3)
        self.steps_per_rev = int(360 / self.deg_per_step)  # 4096

        self.microsteps = 32
        self.delay = 0.005/self.microsteps

    def enable(self, enable):
        GPIO.output(self.enablePin, not enable)

    def step(self, steps, direction):
        GPIO.output(self.dirPin, direction)
        for i in range(steps):
            GPIO.output(self.stepPin, GPIO.HIGH)
            sleep(self.delay)
            GPIO.output(self.stepPin, GPIO.LOW)
            sleep(self.delay)

        print("stepperDriver complete (turned " + dir + " " + str(steps) + " steps)")

    def setSpeed(self, rpm):
        pass
     
"""

"""
motor = StepperMotor(pins=[11, 15, 16, 18])
"""
class StepperMotor(object):

    Seq = [(1, 0, 0, 1),
           (1, 0, 0, 0),
           (1, 1, 0, 0),
           (0, 1, 0, 0),
           (0, 1, 1, 0),
           (0, 0, 1, 0),
           (0, 0, 1, 1),
           (0, 0, 0, 1)]

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    def __init__(self, pins):
        self.pins = pins

        GPIO.setup(self.pins, GPIO.OUT)
        self.clearPins()

        self.stepCount = 0

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

        self.deg_per_step = 360.0 / steps_per_rev      # degree per step

    def rotate(self, degree=90.0, clockwise=False, rpm=6, step_type="Half", verbose=False, init_delay=.05):
        """ motor_go,  moves stepper motor based on 6 inputs
         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to
         step motor 5 options
            (Full, Half, 1/4, 1/8, 1/16)
         (3) steps, type=int, default=200, help=Number of steps sequence's
         to execute. Default is one revolution , 200 in Full mode.
         (4) stepdelay, type=float, default=0.05, help=Time to wait
         (in seconds) between steps.
         (5) verbose, type=bool  type=bool default=False
         help="Write pin actions",
         (6) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.
        """
        try:
            self.set_resolution(step_type)

            sleep(init_delay)

            # Calculate time between steps in seconds
            step_delay = 60.0 / (self.steps_per_rev * rpm)
            
            # Convert degrees to steps
            numberOfSteps = self.deg2Steps(degree)

            print("step_delay: {}, # of steps: {}".format(step_delay, numberOfSteps))

            if not clockwise:
                self.pins.reverse()

            for _ in range(0, numberOfSteps):
                self.step()
                sleep(step_delay)
                if verbose:
                    print("stepCount: {}, stepsToDeg: {:.2f}".format(self.stepCount, self.steps2Deg(self.stepCount)))
                #self.simulateEncoder()
            
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

    """
    def simulateEncoder(self):
        new_msg = Int64()
        new_msg.data = self.stepCount
        self.pub_counter.publish(new_msg)
    """