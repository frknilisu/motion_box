from os import system

camera = Camera()
camera.resolution = (1024, 768)
for i in range(10):
	camera.take_picture('image{0:04d}.jpg'.format(i))
	sleep(60)

system('convert -delay 10 -loop 0 image*.jpg animation.gif')
print('done')