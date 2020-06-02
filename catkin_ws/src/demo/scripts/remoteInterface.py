
import rospy
from std_msgs.msg import Int64
from demo.msg import TimelapseMsg, CmdStepMsg


if __name__ == "__main__":
    rospy.init_node('remoteInterface')

    pub_timelapse = rospy.Publisher(
        "/cmd_timelapse", TimelapseMsg, queue_size=10)
    pub_simplemove = rospy.Publisher(
        "/cmd_simplemove", CmdStepMsg, queue_size=10)

    rospy.spin()
