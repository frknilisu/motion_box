import time
from demo.msg import CmdStepMsg
from geometry_msgs.msg import Twist


class VideoTimelapse:

    def __init__(self):
        pass

    def run(self, video_length, degree, direction):
        interval_delay = 0
        # step_delay = (video_length*motor.stepSize) / (degree)
        # degree to rev
        speed = (60.0 / video_length) * (degree / 360.0)
        # test conversion of video length and angle to rpm
        assert(test(video_length=10, degree=360), 6)
        # TODO: start video recording via /start_record (might be service|action)
        # TODO: trigger motor step to turn until interval_angle via /cmd_step possibly in loop (use speed and direction)
        # TODO: stop video recording via /stop_record (might be service|action)


class PhotoTimelapse:
    def __init__(self, pub):
        self.pub_cmd_step = pub

    def run(self, record_duration, video_length, fps, degree, direction):
        start_time = time.time()

        no_of_photo = fps * video_length
        interval_degree = degree / no_of_photo
        interval_delay = (record_duration / no_of_photo)/1000
        speed = 0.0  # TODO: calculate speed

        for i in range(no_of_photo):
            print('image{0:04d}.jpg'.format(i))
            # TODO: trigger dslr to capture photo via /take_shot

            new_msg = CmdStepMsg()
            new_msg.speed = speed
            new_msg.degree = interval_degree
            new_msg.direction = direction
            self.pub_cmd_step.publish(new_msg)

            time.sleep(interval_delay)

        #system('convert -delay 10 -loop 0 image*.jpg animation.gif')
        print('done')
        elapsed_time = time.time() - start_time
        print("Elapsed Time: ", elapsed_time)
