#! /usr/bin/python3

import rospy
import json
import signal
import sys

from std_msgs.msg import String
from motion_box.srv import StringTrigger, StringTriggerRequest

from threading import Thread
from flask import Flask, render_template, request, redirect, jsonify

Thread(target=lambda: rospy.init_node('remote_listener', disable_signals=True)).start()

pub_mission = rospy.Publisher('mission', String, queue_size=1)

rospy.wait_for_service('motor/drive/action')
motor_drive_action_srv = rospy.ServiceProxy('motor/drive/action', StringTrigger)

pub_joy_cmd_vel = rospy.Publisher('joystick/cmd_vel', String, queue_size=1)

app = Flask(__name__, template_folder='../templates')

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/timelapse", methods=["POST"])
def timelapse():
    d = request.form.to_dict()
    print(d)

    record_time = float(d["record_time"])
    play_time = float(d["play_time"])
    fps = int(d["fps"])
    no_of_photos = int(d["no_of_photos"])
    interval_time = float(d["interval_time"])
    video_speed = int(d["video_speed"])
    degree = int(d["degree"])
    direction = True if d["direction"] == "True" else False

    data = json.dumps({
        'command': 'start', 
        'record_time': record_time, 
        'play_time': play_time, 
        'fps': fps, 
        'no_of_photos': no_of_photos, 
        'interval_time': interval_time, 
        'video_speed': video_speed, 
        'degree': degree, 
        'direction': direction
    })
    print(data)
    pub_mission.publish(data)

    return redirect('/')

@app.route("/test", methods=["POST"])
def test():
    d = request.form.to_dict()
    print(d)

    step_number = int(d["step_number"])
    direction = True if d["direction"] == "True" else False
    step_type = str(d["step_types"]).split()[0]
    speed = float(d["speed"])
    
    data = json.dumps({
        'command': 'move', 
        'steps': step_number, 
        'direction': direction,  
        'step_type': step_type, 
        'speed': speed
    })
    print(data)
    motor_drive_action_msg = StringTriggerRequest(data=data)
    result = motor_drive_action_srv(motor_drive_action_msg)
    print(result)

    return redirect('/')

@app.route("/joy", methods=["POST"])
def joy():
    d = request.form.to_dict()
    print(d)

    vx = int(d["vx"])
    vy = int(d["vy"])
    step_number = int(d["step_number"])
    direction = True if d["direction"] == "True" else False
    step_type = str(d["step_types"]).split()[0]
    speed = float(d["speed"])
    
    data = json.dumps({
        'command': 'move', 
        'steps': step_number, 
        'direction': direction,  
        'step_type': step_type, 
        'speed': speed
    })
    print(data)
    pub_joy_cmd_vel.publish(data)
    
    return redirect('/')

def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

# Run the app on the local development server
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=True)
