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
#     "command": "rotate",
#     "degree": 30.0,
#     "direction": True
#     "step_type": "Full", 
#     "speed": 3.0
# }

# {
#     'command': 'move', 
#     'steps': 150,
#     'direction': False,  
#     'step_type': "Half", 
#     'speed': 0.5
# }

# }
def motorCmd_cb(request):
    rospy.loginfo(rospy.get_caller_id() + ": motorCmd_cb()..")
    data = json.loads(request.data)
    rospy.logdebug(rospy.get_caller_id() + ": data = {}".format(str(data)))
    handleCommand(data)
    return StringTriggerResponse(
        success=True,
        message="I reached there!"
    )

def handleCommand(data):
    if data['command'] == "move":
        moveCommand(data)
    elif data['command'] == "rotate":
        rotateCommand(data)

def moveCommand(data):
    steps = data['steps']
    direction = data['direction']
    step_type = data['step_type']
    rpm = data['speed']
    rospy.loginfo(rospy.get_caller_id() + ": Move (Steps: {}, Direction: {}, StepType: {}, Speed: {})".format(steps, direction, step_type, rpm))

    motor_controller.setMicrostep(microstep=step_type)
    motor_controller.setDirection(clockwise=direction)
    motor_controller.setRPM(rpm=rpm)
    motor_controller.move(steps=steps)

def rotateCommand(data):
    degree = data['degree']
    direction = data['direction']
    step_type = data['step_type']
    rpm = data['speed']
    rospy.loginfo(rospy.get_caller_id() + ": Rotate (Degree: {}, Direction: {}, StepType: {}, Speed: {})".format(degree, direction, step_type, rpm))

    motor_controller.setMicrostep(microstep=step_type)
    motor_controller.setDirection(clockwise=direction)
    motor_controller.setRPM(rpm=rpm)
    motor_controller.rotate(degree=degree)

if __name__ == "__main__":
    rospy.init_node('motorController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    mode_pins = (14, 15, 18)     # Microstep Resolution MS1-MS3 -> GPIO Pin
    step_pin = 21                # Step -> GPIO Pin
    direction_pin = 20           # Direction -> GPIO Pin
    enable_pin = 16
    motor_controller = A4988_Nema(step_pin, direction_pin, enable_pin, mode_pins, "DRV8825")

    #motor_controller = ULN2003A_BYJ([17, 22, 23, 24])

    rospy.Service("motor/cmd", StringTrigger, motorCmd_cb)

    rospy.spin()
