import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

control_pins = [7, 11, 13, 15]

def clearPins():
	for pin in control_pins:
	    GPIO.setup(pin, GPIO.OUT)
	    GPIO.output(pin, 0)

clearPins()

def setStep(seq):
	for i in range(4):
		GPIO.output(control_pins[i], seq[i])

half_step_seq = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]
]

single_step_seq = [
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1]
]

full_step_seq = [
    [1,1,0,0],
    [0,1,1,0],
    [0,0,1,1],
    [1,0,0,1]
]

seq = full_step_seq

for i in range(32*16):
	for j in range(len(seq)):
	    setStep(seq[j])
	    time.sleep(0.01)

GPIO.cleanup()