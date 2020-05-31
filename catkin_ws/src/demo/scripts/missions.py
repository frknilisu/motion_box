from demo.msg import CmdStepMsg


class VideoTimelapse:

    def __init__(self):
        pass

    def run(self, video_length, angle, direction):
        interval_delay = 0
        # step_delay = (video_length*motor.stepSize) / (degree)
        # degree to rev
        speed = (60.0 / video_length) * (angle / 360.0)
        # test conversion of video length and angle to rpm
        assert(test(video_length=10, angle=360), 6)
        # TODO: start video recording via /start_record (might be service|action)
        # TODO: trigger motor step to turn until interval_angle via /cmd_step possibly in loop (use speed and direction)
        # TODO: stop video recording via /stop_record (might be service|action)


class PhotoTimelapse:
    def __init__(self):
        pass

    def run(self, record_duration, video_length, fps, angle, direction):
        start_time = time.time()

        no_of_photo = fps * video_length
        interval_angle = angle / no_of_photo
        interval_delay = (record_duration / no_of_photo)/1000
        # TODO: calculate speed

        for i in range(no_of_photo):
            print('image{0:04d}.jpg'.format(i))
            # TODO: trigger dslr to capture photo via /take_shot
            # TODO: trigger motor step to turn until interval_angle via /cmd_step
            time.sleep(interval_delay)

        #system('convert -delay 10 -loop 0 image*.jpg animation.gif')
        print('done')
        elapsed_time = time.time() - start_time
        print("Elapsed Time: ", elapsed_time)


class SimpleMove:
    def __init__(self):
        pass

    def run(self, speed, angle, direction):
        numberOfSteps = self.deg2Steps(angle)
        for _ in range(0, numberOfSteps):
            # TODO: publish cmd_step with speed and direction
            pass