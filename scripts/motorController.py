
#!/usr/bin/python3

import sys
import time
import random
import struct

import rospy
from std_msgs.msg import Int64, Bool
from motion_box.msg import CmdStepMsg


def encoder_counter_cb(self, msg):
    self.last_encoder_counter = msg.data


def wait_for(self, condition, timeout=None, interval=0.1, errmsg=None):
    '''Wait for a condition to be True.
    Wait for condition, a callable, to return True.  If timeout is
    nonzero, raise a TimeoutError(errmsg) if the condition is not
    True after timeout seconds.  Check the condition everal
    interval seconds.
    '''

    t0 = time.time()
    while not condition():
        t1 = time.time()
        if timeout and (t1 - t0) >= timeout:
            raise TimeoutError(errmsg)

        time.sleep(interval)


def wait_for_degree(self, degree, timeout=None):
    try:
        self.wait_for(lambda: self.last_encoder_counter == degree, timeout=timeout)
    except TimeoutError:
        return False

    return True


def simplemove_cb(self, msg):
    pub_cmd_step.publish(msg)
    resp = wait_for_degree(msg.degree, timeout=(msg.degree / 6*msg.speed))
    pub_move_feedback.publish(resp)


if __name__ == "__main__":
    try:
        rospy.init_node('motorController', log_level=rospy.DEBUG)
        rate = rospy.Rate(​1.0)

        pub_cmd_step = rospy.Publish("/cmd_step", CmdStepMsg, queue_size=10)
        pub_move_feedback = rospy.Publisher("/move_done", Bool, queue_size=10)

        rospy.Subscriber("/simple_move", CmdStepMsg, simplemove_cb)

        self.last_encoder_counter = 0
        rospy.Subscriber("/encoder_counter", Int64, self.encoder_counter_cb)

        while not​ rospy.is_shutdown():
            rospy.logdebug(​"rospy loop")​
            rate.sleep()
            # rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
