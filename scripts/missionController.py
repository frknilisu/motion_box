#!/usr/bin/python3

import rospy

from std_msgs.msg import String

from missions import VideoTimelapse, PhotoTimelapse

def mission_cb(msg):
    rospy.loginfo("mission_cb()..")
    motor_cmd_msg = msg
    #VideoTimelapse(pub_motor_cmd).run(msg)

if __name__ == "__main__":
    rospy.init_node('missionController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    pub_motor_cmd = rospy.Publisher("motor/cmd", String, queue_size=10)
    rospy.Subscriber("mission", String, mission_cb)

    mission_controller = PhotoTimelapse(pub_motor_cmd)
    mission_controller.run(motor_cmd_msg)

    rospy.spin()
