from os import system
import time

import rospy
from std_msgs.msg import Int64
from motion_box.msg import TimelapseMsg, CmdStepMsg
from missions import VideoTimelapse, PhotoTimelapse


def timelapse_cb(msg):
    fps = msg.fps
    record_duration = msg.record_duration
    video_length = msg.video_length
    degree = msg.degree
    direction = msg.direction
    PhotoTimelapse(pub_cmd_step).run(record_duration,
                                     video_length, fps, degree, direction)
    #VideoTimelapse(pub_cmd_step).run(video_length, degree, direction)

if __name__ == "__main__":
    try:
        rospy.init_node('missionController', log_level=rospy.DEBUG)
        rate = rospy.Rate(​1.0)

        pub_cmd_step = rospy.Publish("/simple_move", CmdStepMsg, queue_size=10)

        rospy.Subscriber("/timelapse", TimelapseMsg, timelapse_cb)

        while not​ rospy.is_shutdown():
            rospy.logdebug(​"rospy loop")​

            rate.sleep()
            # rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
