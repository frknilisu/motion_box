from os import system
import time
from stepper import StepperMotor

motor = StepperMotor((7,11,13,15))
motor.setup()


def start(record_time, play_time, fps, angle, direction):
	start_time = time.time()
	
	no_of_photo = fps * play_time
	interval_angle = angle / no_of_photo
	interval_delay = (record_time / no_of_photo)/1000
	
	motor.direction = direction

	#camera = Camera()
	#camera.resolution = (1024, 768)
	for i in range(no_of_photo):
		print('image{0:04d}.jpg'.format(i))
		#camera.take_picture('image{0:04d}.jpg'.format(i))
		motor.turnToAngle(interval_angle)
		time.sleep(interval_delay)

	#system('convert -delay 10 -loop 0 image*.jpg animation.gif')
	print('done')
	elapsed_time = time.time() - start_time
	print("Elapsed Time: ", elapsed_time)

if __name__ == "__main__":
	fps = 2
	duration = 10 #second
	angle = 180 #degree
	video_length = 5 #second
	direction = True #CW
	start(duration, video_length, fps, angle, direction)


"""
start_time = time.time()


video_length = 10 #second
angle = 180 #degree

interval_delay = 0
delay = (video_length*0.0879) / (180)
print(delay)
motor.delay = delay
motor.turnToAngle(angle)

print('done')


elapsed_time = time.time() - start_time
print(elapsed_time)
"""
