#!/usr/bin/python3

import rospy

from motion_box.msg import TimelapseMsg, RotateMsg
from networks import Server, Client

def parseData(data):
    rospy.logdebug(data)

    record_duration = int(data.get("record_duration", 10))
    video_length = int(data.get("video_length", 10))
    fps = int(data.get("fps", 25))
    degree = float(data.get("degree", 180.0))
    direction = bool(data.get("direction", True))
    
    msg = TimelapseMsg()
    msg.record_duration = rospy.Time(record_duration)
    msg.video_length = rospy.Time(video_length)
    msg.fps = fps
    msg.degree = degree
    msg.direction = direction
    return msg

if __name__ == "__main__":
    try:
        rospy.init_node('remoteInterface', log_level=rospy.DEBUG)
        r = rospy.Rate(10) # 10Hz

        pub_timelapse = rospy.Publisher(
            "ri/cmd/timelapse", TimelapseMsg, queue_size=10)
        pub_rotate = rospy.Publisher(
            "ri/cmd/rotate", RotateMsg, queue_size=10)

        server = Server(host='0.0.0.0', port=65432)

        while not rospy.is_shutdown():
            server.accept()
            json_data = server.recv()
            msg = parseData(json_data)
            if type(msg) == TimelapseMsg:
                pub_timelapse.publish(msg)
            elif type(msg) == RotateMsg:
                pub_rotate.publish(msg)
            r.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
