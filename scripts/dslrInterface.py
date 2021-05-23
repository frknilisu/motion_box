#!/usr/bin/python3

import rospy
from std_msgs import Empty
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
ShutterPin = 25
GPIO.setup(ShutterPin, GPIO.OUT)

def capture():
    rospy.loginfo('Capturing image')
    GPIO.output(ShutterPin, True)
    #time.sleep(0.5)
    GPIO.output(ShutterPin, False)

def take_shot_cb(msg):
    rospy.loginfo("take_shot_cb()..")
    capture()

if __name__ == "__main__":
    rospy.init_node('dslrInterface', log_level=rospy.DEBUG)

    rospy.Subscriber("/take_shot", Empty, take_shot_cb)

    rospy.spin()