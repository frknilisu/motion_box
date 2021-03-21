#!/usr/bin/python3

import rospy
import json

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

from RpiMotorLib import RpiMotorLib

CW, CCW = (False, True)

def degToSteps(deg, steptype):
    microsteps =  {'Full': 1,
                    'Half': 2,
                    '1/4': 4,
                    '1/8': 8,
                    '1/16': 16,
                    '1/32': 32}
    return int(round((deg / 1.8) * microsteps[steptype]))

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
    # Unpack JSON message
    data = json.loads(request.data)
    parameters = data['parameters']
    rotateCommand(parameters['degree'], parameters['direction'])
    return EmptyResponse()

def rotateCommand(degree, direction, steptype="1/32"):
    rospy.loginfo(rospy.get_caller_id() + ": Rotate {} degree in {} direction".format(degree, direction))
    
    steps = degToSteps(degree)

    # Run the stepper if the direction is appropriate.
    if (direction == "cw"):
        motor_controller.motor_go(clockwise=CW, 
                    steptype=steptype, 
                    steps=steps, 
                    stepdelay=.005, 
                    verbose=True, 
                    initdelay=.05)
    elif (run.direction == "ccw"):
        motor_controller.motor_go(clockwise=CCW, 
                    steptype=steptype, 
                    steps=steps, 
                    stepdelay=.005, 
                    verbose=True, 
                    initdelay=.05)

if __name__ == "__main__":
    rospy.init_node('motorController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    GPIO_pins = (14, 15, 18)    # Microstep Resolution MS1-MS3 -> GPIO Pin
    directionPin = 20           # Direction -> GPIO Pin
    stepPin = 21                # Step -> GPIO Pin
    motor_controller = RpiMotorLib.A4988Nema(directionPin, stepPin, GPIO_pins, "DRV8825")

    rospy.Service("motor/cmd", String, motorCmd_cb)

    rospy.spin()
