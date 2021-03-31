#!/usr/bin/python3

import rospy
import json

from std_msgs.msg import String

from missions import VideoTimelapse, PhotoTimelapse

mission = None

def mission_photo_timelapse_cb(msg):
    global mission
    
    rospy.loginfo(rospy.get_caller_id() + ": mission_photo_timelapse_cb()..")
    data = json.loads(msg.data)
    rospy.loginfo(rospy.get_caller_id() + ": data = {}".format(str(data)))
    if data['command'] == "start":
        if mission is None or not mission.is_running:
            mission = PhotoTimelapse(data)
        else:
            rospy.logwarn("Controller is running already")
    elif data['command'] == "stop":
        if not mission is None and mission.is_running:
            mission.stop()
        else:
            rospy.logwarn("Controller is not running")
    else:
        rospy.logwarn("Unknown command")

if __name__ == "__main__":
    rospy.init_node('missionController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    rospy.Subscriber("mission", String, mission_photo_timelapse_cb)

    while not rospy.is_shutdown():
        if not mission is None and mission.state == "ready":
            mission.start()
        
        r.sleep()
