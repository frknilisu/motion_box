#!/usr/bin/python3

import rospy
import json

from std_msgs.msg import String

from networks import Server, Client

if __name__ == "__main__":
    try:
        rospy.init_node('remoteInterface', log_level=rospy.DEBUG)
        r = rospy.Rate(10) # 10Hz

        pub_mission = rospy.Publisher("mission", String, queue_size=10)

        server = Server(host='0.0.0.0', port=65432)

        while not rospy.is_shutdown():
            server.accept()
            json_data = server.recv()
            pub_mission.publish(json_data)
            r.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
