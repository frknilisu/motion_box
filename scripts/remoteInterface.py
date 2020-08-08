#!/usr/bin/python3

import rospy
from std_msgs.msg import Int64
from motion_box.msg import TimelapseMsg, CmdStepMsg
from networks import Server, Client

def parseData(data):
    rospy.logdebug(data)

    record_duration = int(data.get("record_duration", 10))
    video_length = int(data.get("video_length", 10))
    fps = int(data.get("fps", 25))
    degree = int(data.get("degree", 180))
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
