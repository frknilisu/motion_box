#!/usr/bin/python3

import rospy
import json

from std_msgs.msg import String

from RpiMotorLib import RpiMotorLib


# Callback used to translate the received JSON message to a stepper command.
# Expecting something like this (example for the run command):
# {
#    "command" : "run",
#    "parameters" : {
#        "direction" : "clockwise",
#        "speed" : 800.0
#    }
#}
#PhotoTimelapse(pub_rotate).run(record_duration, video_length, fps, degree, direction)


def motorCmd_cb(msg):
    try:
        # Unpack JSON message
        data = json.loads(msg.data)

        # Execute the given command with its parameters.
        command = data.get('command', None)
        if command == "run":
            parameters = data['parameters']
            runCommand(parameters['direction'], parameters['speed'])
        elif command == "move":
            parameters = data['parameters']
            moveCommand(parameters['direction'], parameters['steps'])
        elif command == "goTo":
            parameters = data['parameters']
            goToCommand(parameters['position'])
        elif command == "rotate":
            parameters = data['parameters']
            rotateCommand(parameters['degree'], parameters['direction'])
        elif command == "stop":
            parameters = data['parameters']
            stopCommand(parameters['type'])
        else:
            rospy.logerr(rospy.get_caller_id() + ": Unrecognized command \"%s\"" % (command))
    except ValueError as e:
        rospy.logerr(rospy.get_caller_id() + ": Error decoding JSON \"%s\"" % (str(e)))


def degToSteps(deg, steptype):
    microsteps =  {'Full': 1,
                    'Half': 2,
                    '1/4': 4,
                    '1/8': 8,
                    '1/16': 16,
                    '1/32': 32}
    return int(round((deg / 1.8) * microsteps[steptype]))

DIR_CLOCKWISE = False
DIR_COUNTER_CLOCKWISE = True

def rotateCommand(degree, direction, steptype="1/32"):
    if (direction == "clockwise" or direction == "counter-clockwise"):
        rospy.loginfo(rospy.get_caller_id() + ": Run at %d step/s in %s rotation" % (speed, direction))
    else:
        rospy.logerr(rospy.get_caller_id() + ": Unrecognized run direction for \"%s\"" % (direction))
    
    steps = degToSteps(degree)

    # Run the stepper if the direction is appropriate.
    if (direction == "clockwise"):
        motor_controller.motor_go(clockwise=DIR_CLOCKWISE, 
                    steptype=steptype, 
                    steps=steps, 
                    stepdelay=.005, 
                    verbose=True, 
                    initdelay=.05)
    elif (run.direction == "counter-clockwise"):
        motor_controller.motor_go(clockwise=DIR_COUNTER_CLOCKWISE, 
                    steptype=steptype, 
                    steps=steps, 
                    stepdelay=.005, 
                    verbose=True, 
                    initdelay=.05)

if __name__ == "__main__":
    rospy.init_node('motorController', log_level=rospy.DEBUG)
    r = rospy.Rate(10) # 10Hz

    motor_controller = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "DRV8825")

    rospy.Subscriber("motor/cmd", String, motorCmd_cb)

    rospy.spin()
