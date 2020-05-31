
import rospy
from std_msgs.msg import Int64
from demo.msg import TimelapseMsg, SimpleMoveMsg


if __name__ == "__main__":
    rospy.init_node('remoteInterface')

    pub_timelapse = rospy.Publisher("/cmd_timelapse", TimelapseMsg, queue_size=10)
    pub_simplemove = rospy.Publisher("/cmd_simplemove", SimpleMoveMsg, queue_size=10)

    rospy.spin()