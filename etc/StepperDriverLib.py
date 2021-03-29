import RPi.GPIO as GPIO
from time import sleep

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

    def setSpeed(self, rpm):
        pass
        


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
        return int(round(deg/self.stepSize))

    def steps2Deg(self, steps):
        return float(steps * self.stepSize)

    # void stop()
    # void currentState(float * position)

    def __init__(self, pins):
        self.pins = pins
        self.stepPin = self.pins[0]
		self.directionPin = self.pins[1]
		self.enablePin = self.pins[2]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.pins, GPIO.OUT)
        self._clearPins()

        self.delay = .005/32

        self.deg_per_step = 5.625 / 64  # for half-step drive (mode 3)
        self.steps_per_rev = int(360 / self.deg_per_step)  # 4096

        self.stepCount = 0
        self.seq_head = 0
        self.step_delay = 0.1

        print("Stepper initialized")
        
    def enable(self, enable):
        #set enable to high (i.e. power is NOT going to the motor)
		GPIO.output(self.enablePin, not enable)

    def _clearPins(self):
        GPIO.output(self.pins, (False, )*len(self.pins))

    def run(self, steps, clockwise):
        GPIO.output(self.directionPin, clockwise)
        for i in range(steps):
            GPIO.output(self.stepPin, GPIO.HIGH)
            #sleep(self.delay)
            GPIO.output(self.stepPin, GPIO.LOW)
            sleep(self.delay)
            self.stepCount += 1

        print("stepperDriver complete (turned " + dir + " " + str(steps) + " steps)")

    def callback_cmd_step(self, msg):
        speed = msg.speed
        degree = msg.degree
        direction = msg.direction
        self.stepCount = 0
        self.move(speed, degree, direction)

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
        print("step_delay: {}, # of steps: {}".format(self.step_delay, numberOfSteps))
        for _ in range(0, numberOfSteps):
            self.step()
            self.simulateEncoder()

    def simulateEncoder(self):
        new_msg = Int64()
        new_msg.data = self.stepCount
        self.pub_counter.publish(new_msg)