from flask import Flask, render_template, render_template_string, request
from stepper import StepperMotor
app = Flask(__name__)

motor = StepperMotor((7,11,13,15))
motor.setup()


@app.route("/")
def index():
    return render_template("test_slider.html")

@app.route("/test", methods=["POST"])
def test():
    # Get slider Values
    slider = int(request.form["slider"])
    print(slider)
  
    if (slider>0):
        print("Rotating Clockwise")
        motor.direction = True
        motor.turnToAngle(slider)

    elif (slider<0):
        print("Rotating Anti-Clockwise")
        motor.direction = False
        motor.turnToAngle(-slider)

    return render_template("test_slider.html")
 
# Run the app on the local development server
if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)