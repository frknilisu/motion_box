#!/usr/bin/python3

import rospy
from std_msgs.msg import Int64
from motion_box.msg import TimelapseMsg, CmdStepMsg
from networks import Server, Client

def parseData(data):
    rospy.logdebug(data)
    #angle = data.get("angle", 180)
    msg = TimelapseMsg()
    return msg

if __name__ == "__main__":
    try:
        rospy.init_node('remoteInterface', log_level=rospy.DEBUG)
        r = rospy.Rate(10) # 10Hz

        pub_timelapse = rospy.Publisher(
            "/cmd_timelapse", TimelapseMsg, queue_size=10)
        pub_simplemove = rospy.Publisher(
            "/cmd_simplemove", CmdStepMsg, queue_size=10)

        server = Server(host='0.0.0.0', port=65432)

        while not rospy.is_shutdown():
            server.accept()
            json_data = server.recv()
            msg = parseData(json_data)
            if type(msg) == TimelapseMsg:
                pub_timelapse.publish(msg)
            elif type(msg) == CmdStepMsg:
                pub_simplemove.publish(msg)
            r.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
