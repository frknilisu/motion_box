#! /usr/bin/python3

import rospy
import json
import signal
import sys

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from motion_box.srv import StringTrigger, StringTriggerRequest

from threading import Thread
from flask import Flask, render_template, render_template_string, request, redirect, jsonify

def step2Degree(step_type, steps):
    microsteps =  {'Full': 1,
                    'Half': 2,
                    '1/4': 4,
                    '1/8': 8,
                    '1/16': 16,
                    '1/32': 32}
    steps_per_rev = 200 * microsteps[step_type]
    deg_per_step = 360.0 / steps_per_rev
    return float(steps * deg_per_step)

Thread(target=lambda: rospy.init_node('remote_listener', disable_signals=True)).start()
pub = rospy.Publisher('mission', String, queue_size=1)

#rospy.wait_for_service('motor/cmd')
#motor_cmd_service = rospy.ServiceProxy('motor/cmd', StringTrigger)

app = Flask(__name__, template_folder='../templates')

@app.route("/")
def index():
    return render_template("test.html")

@app.route("/test", methods=["POST"])
def test():
    d = request.form.to_dict()
    print(d)

    step_type = str(d["step_types"]).split()[0]
    d["step_types"] = step_type

    step_number = int(d["step_number"])
    d["step_number"] = step_number
    
    speed = float(d["speed"])
    d["speed"] = speed
    
    direction = True if d["direction"] == "True" else False
    d["direction"] = direction

    print(d)
    
    data = json.dumps({
        'command': 'rotate', 
        'degree': step2Degree(d['step_types'], d['step_number']), 
        'direction': d['direction']
    })

    print(data)
    #motor_cmd_msg = StringTriggerRequest(data=data)
    #result = motor_cmd_service(motor_cmd_msg)

    return redirect('/')

def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

# Run the app on the local development server
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=True)
