#!/usr/bin/python3

import rospy
import json

from std_msgs.msg import String

from missions import VideoTimelapse, PhotoTimelapse

controller = None

def mission_photo_timelapse_cb(msg):
    global controller
    
    rospy.loginfo("mission_photo_timelapse_cb()..")
    data = json.loads(msg.data)
    print(data)
    if data['command'] == "start":
        if controller is None or not controller.is_running:
            controller = PhotoTimelapse(msg)
        else:
            rospy.logwarn("Controller is running already")
    elif data['command'] == "stop":
        if not controller is None and controller.is_running:
            controller.stop()
        else:
            rospy.logwarn("Controller is not running")
    else:
        rospy.logwarn("Unknown command")

if __name__ == "__main__":
    rospy.init_node('missionController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    rospy.Subscriber("mission", String, mission_photo_timelapse_cb)

    while not rospy.is_shutdown():
        if not controller is None and controller.state == "ready":
            controller.start()
        
        r.sleep()
