#! /usr/bin/python3

import rospy
import json
import signal
import sys

from std_msgs.msg import String

from threading import Thread
from flask import Flask, render_template, render_template_string, request, redirect, jsonify

Thread(target=lambda: rospy.init_node('remote_listener', disable_signals=True)).start()
pub = rospy.Publisher('mission', String, queue_size=1)

app = Flask(__name__, template_folder='../templates')

@app.route("/")
def index():
    return render_template("angular_control.html")

@app.route("/timelapse", methods=["POST"])
def timelapse():
    d = request.form.to_dict()

    record_time = float(d["record_time"])
    d["record_time"] = record_time
    play_time = float(d["play_time"])
    d["play_time"] = play_time
    fps = int(d["fps"])
    d["fps"] = fps
    no_of_photos = int(d["no_of_photos"])
    d["no_of_photos"] = no_of_photos
    interval_time = float(d["interval_time"])
    d["interval_time"] = interval_time
    video_speed = int(d["video_speed"])
    d["video_speed"] = video_speed
    degree = int(d["degree"])
    d["degree"] = degree
    direction = True if d["direction"] == "True" else False
    d["direction"] = direction

    d["command"] = "start"
    pub.publish(json.dumps(d))

    return redirect('/')

@app.route("/test", methods=["POST"])
def test():
    d = request.form.to_dict()
    print(d)

def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

# Run the app on the local development server
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=True)
