import sys
print(sys.version)
import time

import timelapse

from flask import Flask, render_template, render_template_string, request, redirect
app = Flask(__name__)





@app.route("/")
def index():
    return render_template("timelapse.html")

@app.route("/timelapse", methods=["POST"])
def f():
    d = request.form.to_dict()
    angle = int(d["angle"])
    fps = int(d["fps"])
    record_time = int(d["record_time"])
    play_time = int(d["play_time"])
    direction = bool(d["direction"])
    timelapse.start(record_time, play_time, fps, angle, direction)

    return redirect('/')


# Run the app on the local development server
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)
