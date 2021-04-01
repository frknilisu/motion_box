#!/usr/bin/python3

import rospy
import json

from std_msgs.msg import String
from motion_box.srv import StringTrigger, StringTriggerResponse

from StepperDriverLib import A4988_Nema, ULN2003A_BYJ

CW, CCW = (False, True)

# Callback used to translate the received JSON message to a stepper command.
# Expecting something like this (example for the run command):
# {
#    "command": "rotate",
#    "degree": 30.0,
#    "direction": True
# }
def motorCmd_cb(request):
    rospy.loginfo(rospy.get_caller_id() + ": motorCmd_cb()..")
    data = json.loads(request.data)
    rospy.logdebug(rospy.get_caller_id() + ": data = {}".format(str(data)))
    rotateCommand(data['degree'], data['direction'])
    return StringTriggerResponse(
        success=True,
        message="I reached there!"
    )

def rotateCommand(degree, direction, step_type="1/32"):
    rospy.loginfo(rospy.get_caller_id() + ": Rotate {} degree in {} direction".format(degree, direction))

    # Run the stepper if the direction is appropriate.
    if (direction == "cw" or direction is False):
        motor_controller.rotate(degree=degree, clockwise=CW)
    elif (direction == "ccw" or direction is True):
        motor_controller.rotate(degree=degree, clockwise=CCW)

if __name__ == "__main__":
    rospy.init_node('motorController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    mode_pins = (14, 15, 18)     # Microstep Resolution MS1-MS3 -> GPIO Pin
    step_pin = 21                # Step -> GPIO Pin
    direction_pin = 20           # Direction -> GPIO Pin
    enable_pin = 16
    #motor_controller = A4988_Nema(step_pin, direction_pin, enable_pin, mode_pins, "DRV8825")

    motor_controller = ULN2003A_BYJ([17, 22, 23, 24])

    rospy.Service("motor/cmd", StringTrigger, motorCmd_cb)

    rospy.spin()
