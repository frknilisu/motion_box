#!/usr/bin/python3

import rospy

from motion_box.msg import TimelapseMsg, RotateMsg
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

        pub_rotate = rospy.Publisher("/rotate", RotateMsg, queue_size=10)

        while not rospy.is_shutdown():
            r.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
