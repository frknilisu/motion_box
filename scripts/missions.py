#!/usr/bin/python3

import rospy
import time

from std_msgs.msg import String

class VideoTimelapse:

    def __init__(self, pub):
        self.pub_motor_cmd = pub

    def run(self, video_length, degree, direction):
        rospy.loginfo("Recording started!..")
        # TODO: start video recording via /start_record (might be service|action)

        #new_msg.duration = video_length
        new_msg.degree = degree
        new_msg.direction = direction
        self.pub_motor_cmd.publish(new_msg)
        
        # TODO: stop video recording via /stop_record (might be service|action)
        rospy.loginfo("Recording stopped!..")


class PhotoTimelapse:
    
    def __init__(self, pub):
        self.pub_motor_cmd = pub

    def run(self, record_duration, video_length, fps, degree, direction):
        start_time = time.time()

        no_of_photo = int(fps * video_length)
        interval_duration = record_duration / no_of_photo
        interval_degree = degree / no_of_photo

        rospy.loginfo("{} photo will be taken at each {} degree by {} sec".format(no_of_photo, interval_degree, interval_duration))

        for i in range(no_of_photo):
            rospy.loginfo('image{0:04d}.jpg'.format(i))

            # TODO: trigger dslr to capture photo via /take_shot

            new_msg = RotateMsg()
            new_msg.degree = interval_degree
            new_msg.direction = direction
            self.pub_motor_cmd.publish(new_msg)

            # TODO: wait/check until reach given degree

            time.sleep(interval_duration)

        #system('convert -delay 10 -loop 0 image*.jpg animation.gif')
        rospy.loginfo('done')
        elapsed_time = time.time() - start_time
        rospy.loginfo("Elapsed Time: {}".format(elapsed_time))
