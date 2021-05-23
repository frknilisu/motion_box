#! /usr/bin/python3

import rospy
import json
import signal
import sys

from std_msgs.msg import String
from motion_box.srv import StringTrigger, StringTriggerRequest

from threading import Thread
from flask import Flask, render_template, render_template_string, request, redirect, jsonify

Thread(target=lambda: rospy.init_node('remote_listener', disable_signals=True)).start()
pub = rospy.Publisher('mission', String, queue_size=1)

rospy.wait_for_service('motor/cmd')
motor_cmd_service = rospy.ServiceProxy('motor/cmd', StringTrigger)

app = Flask(__name__, template_folder='../templates')

@app.route("/")
def index():
    return render_template("test.html")

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
    motor_cmd_msg = StringTriggerRequest(data=data)
    result = motor_cmd_service(motor_cmd_msg)
    print(result)
    return redirect('/')

def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

# Run the app on the local development server
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=True)
