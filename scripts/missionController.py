#!/usr/bin/python3

from os import system
import time

import rospy
from std_msgs.msg import Int64
from motion_box.msg import TimelapseMsg, CmdStepMsg
from missions import VideoTimelapse, PhotoTimelapse


def timelapse_cb(msg):
    fps = msg.fps
    record_duration = msg.record_duration.to_sec()
    video_length = msg.video_length.to_sec()
    degree = msg.degree
    direction = msg.direction
    PhotoTimelapse(pub_cmd_step).run(record_duration,
                                     video_length, fps, degree, direction)
    #VideoTimelapse(pub_cmd_step).run(video_length, degree, direction)

if __name__ == "__main__":
    try:
        rospy.init_node('missionController', log_level=rospy.DEBUG)
        r = rospy.Rate(10) # 10Hz

        pub_cmd_step = rospy.Publisher("/simple_move", CmdStepMsg, queue_size=10)

        rospy.Subscriber("/timelapse", TimelapseMsg, timelapse_cb)

        while not rospy.is_shutdown():
            # ... do some work ...
            r.sleep()
        #rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
