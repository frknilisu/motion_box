#!/usr/bin/python3

import sys
import time
import random
import struct

import rospy
from std_msgs.msg import Int64, Bool
from motion_box.msg import CmdStepMsg


def encoder_counter_cb(msg):
    global last_encoder_counter
    last_encoder_counter = msg.data


def wait_for(condition, timeout=None, interval=0.1, errmsg=None):
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


def wait_for_degree(steps, timeout=None):
    try:
        wait_for(lambda: last_encoder_counter == steps, timeout=timeout)
    except TimeoutError:
        return False

    return True

def deg2Steps(deg):
    return int(round(deg/(360/4096)))

def simplemove_cb(msg):
    global timerStart, lastMsg
    timerStart = True
    lastMsg = msg
    pub_cmd_step.publish(msg)


if __name__ == "__main__":
    try:
        rospy.init_node('motorController', log_level=rospy.DEBUG)
        r = rospy.Rate(10) # 10Hz
        timerStart = False
        lastMsg = CmdStepMsg()

        pub_cmd_step = rospy.Publisher("/cmd_step", CmdStepMsg, queue_size=10)
        pub_move_feedback = rospy.Publisher("/move_done", Bool, queue_size=10)

        rospy.Subscriber("/simple_move", CmdStepMsg, simplemove_cb)

        last_encoder_counter = 0
        rospy.Subscriber("/encoder_counter", Int64, encoder_counter_cb)

        
        while not rospy.is_shutdown():
            #print("timerStart: {}, lastMsg: {}, encoder: {}".format(timerStart, deg2Steps(lastMsg.degree), last_encoder_counter))
            if timerStart and deg2Steps(lastMsg.degree) == last_encoder_counter: #(msg.degree / 6*msg.speed))
                pub_move_feedback.publish(True)
                timerStart = False
            # ... do some work ...
            r.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
