#!/usr/bin/python3

import rospy
import json

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from motion_box.srv import StringTrigger, StringTriggerRequest, StringTriggerResponse

from RpiMotorLib import RpiMotorLib
from StepperDriverLib import StepperMotor

CW, CCW = (False, True)

# 'run': params('direction', 'speed')
# 'move': params('direction', 'steps')
# 'goTo': params('position')
# 'rotate': params('direction', 'degree')
# 'stop': params('type')

# Callback used to translate the received JSON message to a stepper command.
# Expecting something like this (example for the run command):
# {
#    "command" : "run",
#    "parameters" : {
#        "direction" : "clockwise",
#        "speed" : 800.0
#    }
#}
def motorCmd_cb(request):
    rospy.loginfo("motorCmd_cb()..")
    # Unpack JSON message
    data = json.loads(request.data)
    print(data)
    rotateCommand(data['degree'], data['direction'])
    return StringTriggerResponse(
        success=True,
        message="Hey, roger that; we'll be right there!"
    )

def rotateCommand(degree, direction, step_type="1/32"):
    rospy.loginfo(rospy.get_caller_id() + ": Rotate {} degree in {} direction".format(degree, direction))

    # Run the stepper if the direction is appropriate.
    if (direction == "cw"):
        """
        motor_controller.motor_go(clockwise=CW, 
                    steptype=steptype, 
                    steps=steps, 
                    stepdelay=.005, 
                    verbose=True, 
                    initdelay=.05)
        """
        motor_controller.rotate(degree=degree, clockwise=CW)
    elif (direction == "ccw"):
        """
        motor_controller.motor_go(clockwise=CCW, 
                    steptype=steptype, 
                    steps=steps, 
                    stepdelay=.005, 
                    verbose=True, 
                    initdelay=.05)
        """
        motor_controller.rotate(degree=degree, clockwise=CCW)

if __name__ == "__main__":
    rospy.init_node('motorController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    GPIO_pins = (14, 15, 18)    # Microstep Resolution MS1-MS3 -> GPIO Pin
    directionPin = 20           # Direction -> GPIO Pin
    stepPin = 21                # Step -> GPIO Pin
    #motor_controller = RpiMotorLib.A4988Nema(directionPin, stepPin, GPIO_pins, "DRV8825")

    motor_controller = StepperMotor([11, 15, 16, 18])

    rospy.Service("motor/cmd", StringTrigger, motorCmd_cb)

    rospy.spin()
