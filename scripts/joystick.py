#!/usr/bin/python3

import rospy
import time
import json
import spidev

from std_msgs.msg import String, Int32MultiArray


axis_thresholds = (0, 500, 523, 1023)
speed_range = (5, 50)
button_threshold = 10

class MCP3008:

    def __init__(self) -> None:

        # Define Axis Channels (channel 3 to 7 can be assigned for more buttons / joysticks)
        self.swt_channel = 0
        self.vrx_channel = 1
        self.vry_channel = 2
        
        # Spi oeffnen
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)
        self.spi.max_speed_hz=1000000
 
    def read(self):
        # Function for reading the MCP3008 channel between 0 and 7
        def readChannel(channel):
            val = self.spi.xfer2([1, (8 + channel) << 4, 0])
            data = ((val[1] & 3) << 8) + val[2]
            return data

        vrx_pos = readChannel(self.vrx_channel)
        vry_pos = readChannel(self.vry_channel)
        swt_val = readChannel(self.swt_channel)

        return (vrx_pos, vry_pos, swt_val)

#  Prominent Arduino map function :)
def map_val(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def checkJoyStatDiff(curr, prev):
    n = len(curr)
    for i in range(n):
        if abs(curr[i] - prev[i]) > 20:
            return True

    return False

if __name__ == "__main__":
    rospy.init_node('joystick', log_level=rospy.DEBUG)
    pub_joy_stat = rospy.Publisher("joystick/status", Int32MultiArray, queue_size=10)
    pub_joy_cmd_vel = rospy.Publisher("joystick/cmd_vel", String, queue_size=10)

    joystick = MCP3008()

    delay = 0.5

    prev_joy_stat = [0, 0, 0]
    curr_joy_stat = [0, 0, 0]

    while True:

        # Determine position
        curr_joy_stat = list(joystick.read())

        # output
        rospy.loginfo("VRx: {}  VRy: {}  SW: {}".format(*curr_joy_stat))

        if curr_joy_stat[0] > 523:
            # map the speed between 5 and 500 rpm
            vx = map_val(curr_joy_stat[0], 523, 1023, 5, 50)
        elif curr_joy_stat[0] <= 500:
            vx = -map_val(curr_joy_stat[0], 500, 0, -5, -50)
        else:
            vx = 0.0

        if curr_joy_stat[1] > 523:
            # map the speed between 5 and 500 rpm
            vy = map_val(curr_joy_stat[1], 523, 1023, 5, 50)
        elif curr_joy_stat[1] <= 500:
            vy = -map_val(curr_joy_stat[1], 500, 0, -5, -50)
        else:
            vy = 0.0
        
        sw_pressed = (curr_joy_stat[2] < 10)

        rospy.logdebug("vx: {}  vy: {}  sw: {}".format(vx, vy, sw_pressed))

        data = json.dumps({
            'vx': vx, 
            'vy': vy, 
            'sw': sw_pressed
        })
        pub_joy_cmd_vel.publish(data)

        if checkJoyStatDiff(curr_joy_stat, prev_joy_stat):
            joy_msg = Int32MultiArray()
            joy_msg.data = curr_joy_stat
            pub_joy_stat.publish(joy_msg)

        prev_joy_stat = curr_joy_stat

        # wait
        time.sleep(delay)

