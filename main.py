import sys
import time
from networks import Server, Client
from stepper import StepperMotor
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot


def callback(data):
    #print("data: %{:.2f}".format(data['percentage']))
    server.send(data)


@pyqtSlot(dict)
def cb_slot(data):
    print("data: %{:.2f}".format(data['percentage']))
    server.send(data)


if __name__ == "__main__":

    server = Server(host='0.0.0.0', port=65432)

    motor = StepperMotor((7, 11, 13, 15), callback)
    motor.cb_signal.connect(cb_slot)
    motor.setup()

    server.accept()
    json_data = server.recv()
    print(type(json_data))
    print(json_data)
    #server.send({'my_key': 'my_value'})

    time.sleep(2.0)

    angle = json_data.get("angle", 180)

    motor.turnToAngle(angle)
