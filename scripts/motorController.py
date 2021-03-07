#!/usr/bin/python3

import rospy

from motion_box.msg import RotateMsg

def rotateMsg_cb(msg):
    rotate(msg.degree, msg.direction)

def degToSteps(deg, steptype):
    microsteps =  {'Full': 1,
                    'Half': 2,
                    '1/4': 4,
                    '1/8': 8,
                    '1/16': 16,
                    '1/32': 32}
    return int(round((deg / 1.8) * microsteps[steptype]))

def rotate(degree, direction, steptype="1/32"):
    steps = degToSteps(degree)
    motor.motor_go(clockwise=direction, 
                    steptype=steptype, 
                    steps=steps, 
                    stepdelay=.005, 
                    verbose=True, 
                    initdelay=.05)

if __name__ == "__main__":
    try:
        rospy.init_node('motorController', log_level=rospy.DEBUG)
        r = rospy.Rate(10) # 10Hz

        motor = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "DRV8825")

        rospy.Subscriber("/rotate", RotateMsg, rotateMsg_cb)

        while not rospy.is_shutdown():
            r.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
