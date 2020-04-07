import RPi.GPIO as GPIO
import time

shots = 100
interval = 1

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT)
taken = 1
for i in range(shots):
	print("Shot {} of {}".format(taken, shots))
	taken+=1
	GPIO.output(16, GPIO.HIGH)
	time.sleep(0.5)
	GPIO.output(16, GPIO.LOW)
	time.sleep(interval)

GPIO.cleanup()