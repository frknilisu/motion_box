#!/usr/bin/python3

from __future__ import print_function

import logging
import os
import subprocess
import sys
import time
import gphoto2 as gp

import rospy
from std_msgs.msg import Empty
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def take_shot_by_GPIO():
    ShutterPin = 25
    FocusPin = 23
    GPIO.setup(FocusPin, GPIO.OUT)
    GPIO.setup(ShutterPin, GPIO.OUT)
    GPIO.output(FocusPin, True)
    time.sleep(0.5)
    GPIO.output(ShutterPin, True)
    time.sleep(0.5)
    GPIO.output(ShutterPin, False)
    GPIO.output(FocusPin, False)

def camera_info():
    camera_list = list(gp.Camera.autodetect())
    name, addr = camera_list[0]
    camera = gp.Camera()
    # search ports for camera port name
    port_info_list = gp.PortInfoList()
    port_info_list.load()
    idx = port_info_list.lookup_path(addr)
    camera.set_port_info(port_info_list[idx])
    camera.init()
    text = camera.get_summary()
    print('Summary')
    print('=======')
    print(str(text))
    try:
        text = camera.get_manual()
        print('Manual')
        print('=======')
        print(str(text))
    except Exception as ex:
        print(str(ex))
    camera.exit()
    return 0


def capture1():
    # time between captures
    INTERVAL = 10.0
    WORK_DIR = '/tmp/time_lapse'
    OUT_FILE = 'timelapse.mp4'

    if not os.path.exists(WORK_DIR):
        os.makedirs(WORK_DIR)
    template = os.path.join(WORK_DIR, 'frame%04d.jpg')
    path = camera.capture(gp.GP_CAPTURE_IMAGE)
    print('capture', path.folder + path.name)
    camera_file = camera.file_get(
        path.folder, path.name, gp.GP_FILE_TYPE_NORMAL)
    camera_file.save(template % count)
    camera.file_delete(path.folder, path.name)
    next_shot += INTERVAL
    count += 1

    subprocess.check_call(
        ['ffmpeg', '-r', '25', '-i', template, '-c:v', 'h264', OUT_FILE])


def capture2():
    logging.basicConfig(
        format='%(levelname)s: %(name)s: %(message)s', level=logging.WARNING)
    callback_obj = gp.check_result(gp.use_python_logging())
    camera = gp.Camera()
    camera.init()
    print('Capturing image')
    file_path = camera.capture(gp.GP_CAPTURE_IMAGE)
    print('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))
    target = os.path.join('/tmp', file_path.name)
    print('Copying image to', target)
    camera_file = camera.file_get(
        file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL)
    camera_file.save(target)
    subprocess.call(['xdg-open', target])
    camera.exit()
    return 0


def callback_take_shot(msg):
    # TODO: call capture()
    print("callback_take_shot")
    take_shot_by_GPIO()

def callback_start_record(msg):
    # TODO: call start_record()
    print("callback_start_record")


if __name__ == "__main__":
    rospy.init_node('dslrInterface', log_level=rospy.DEBUG)
    rate = rospy.Rate(​10.0)

    rospy.Subscriber("/take_shot", Empty, callback_take_shot)
    rospy.Subscriber("/start_record", Empty, callback_start_record)
    # TODO: /stop_record

    rospy.spin()