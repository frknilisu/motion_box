import RPi.GPIO as GPIO
from time import sleep

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

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
motor = StepperMotor(pins=(7, 11, 13, 15))
"""
class StepperMotor:

    Seq = [(1, 0, 0, 1),
           (1, 0, 0, 0),
           (1, 1, 0, 0),
           (0, 1, 0, 0),
           (0, 1, 1, 0),
           (0, 0, 1, 0),
           (0, 0, 1, 1),
           (0, 0, 0, 1)]

    def __init__(self, pins, steps_per_rev=4096.0):
        self.pins = pins

        GPIO.setup(self.pins, GPIO.OUT)
        self.clearPins()

        self.steps_per_rev = steps_per_rev    # steps per revolution
        self.deg_per_step = 360.0 / steps_per_rev      # degree per step
        
        self.angle = 0.0
        self.stepCount = 0

        print("Stepper initialized")

    def setPins(self, seq):
        """Set the pins with given sequence"""
        GPIO.output(self.pins, seq)
    
    def clearPins(self):
        self.setPins((GPIO.LOW, )*len(self.pins))

    def step(self, seq_idx):
        """Take a step in current direction"""
        self.setPins(self.Seq[self.stepCount % len(self.Seq)])
        self.stepCount += 1

    def deg2Steps(self, degree):
        #return int(round(deg/self.stepSize))
        return math.fabs(degree / self.deg_per_step)

    def steps2Deg(self, steps):
        return float(steps * self.deg_per_step)

    def rotate(self, degree=360, rpm=15, direction=False):
        # Calculate time between steps in seconds
        step_delay = 60.0 / (self.steps_per_rev * rpm)
        
        # Convert degrees to steps
        numberOfSteps = self.deg2Steps(degree)

        print("step_delay: {}, # of steps: {}".format(step_delay, numberOfSteps))

        if not direction:
            self.pins.reverse()

        for _ in range(0, numberOfSteps):
            self.step()
            sleep(step_delay)
            self.angle += self.deg_per_step if direction else -self.deg_per_step
            #self.simulateEncoder()
        
        if not direction:
            self.pins.reverse()
    	
        self.clearPins()

    """
    def simulateEncoder(self):
        new_msg = Int64()
        new_msg.data = self.stepCount
        self.pub_counter.publish(new_msg)
    """