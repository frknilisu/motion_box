#!/usr/bin/python3

import rospy
import sys
import time
import json

from std_msgs.msg import Int64, Bool, String
#from sensor_msgs.msg import Joy

import spidev
import os

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


if __name__ == "__main__":
    rospy.init_node('joystick', log_level=rospy.DEBUG)
    pub_joy = rospy.Publisher("joystick_status", String, queue_size=10)

    joystick = MCP3008()

    #Time delay, which tells how many seconds the value is read out
    delay = 0.5

    prev_x = 0
    prev_y = 0
    prev_sw = 0

    curr_x = 0
    curr_y = 0
    curr_sw = 0

    while True:

        # Determine position
        curr_x, curr_y, curr_sw = joystick.read()

        # output
        rospy.loginfo("VRx: {}  VRy: {}  SW: {}".format(curr_x, curr_y, curr_sw))

        if abs(curr_x - prev_x) > 10 or abs(curr_y - prev_y) > 10 or abs(curr_sw - prev_sw) > 25:
            data = json.dumps({
                'x': curr_x, 
                'y': curr_y, 
                'pressed': curr_sw
            })
            joy_msg = String()
            joy_msg.data = data

            pub_joy.publish(joy_msg)


        if curr_x > 523:
            # map the speed between 5 and 500 rpm
            vx = map_val(curr_x, 523, 1023, 5, 500)
        elif curr_x <= 500:
            vx = -map_val(curr_x, 0, 500, 5, 500)
        else:
            vx = 0.0

        if curr_y > 523:
            # map the speed between 5 and 500 rpm
            vy = map_val(curr_y, 523, 1023, 5, 500)
        elif curr_y <= 500:
            vy = -map_val(curr_y, 0, 500, 5, 500)
        else:
            vy = 0.0
        
        sw_pressed = (curr_sw < 10)

        rospy.logdebug("vx: {}  vy: {}  sw: {}".format(vx, vy, sw_pressed))

        prev_x = curr_x
        prev_y = curr_y
        prev_sw = curr_sw

        # wait
        time.sleep(delay)

