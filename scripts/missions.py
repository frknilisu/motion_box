#!/usr/bin/python3

import rospy
import time
import json

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from motion_box.srv import StringTrigger, StringTriggerRequest

from abc import ABC, abstractmethod

class Mission(ABC):
    def __init__(self):
        self._state = "uninitialized"

        rospy.wait_for_service('motor/cmd')
        self.motor_cmd_service = rospy.ServiceProxy('motor/cmd', StringTrigger)

        super().__init__()

    @abstractmethod
    def process(self):
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def checkStopFlags(self):
        pass

    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, state):
        self._state = state

    @state.deleter
    def state(self):
        del self._state

    
class VideoTimelapse(Mission):

    def __init__(self, data):
        super().__init__()

        self.video_length = data.get('video_length', 7200)
        self.degree = data.get('degree', 7200)
        self.direction = data.get('direction', 7200)
        
    def process(self):
        rospy.loginfo("Recording started!..")
        # TODO: start video recording via /start_record (might be service|action)

        motor_cmd_msg = json.dumps({
            'command': 'rotate', 
            'degree': self.degree, 
            'direction': self.direction
        })
        motor_cmd_msg2 = TriggerRequest()
        result = self.motor_cmd_service(motor_cmd_msg2)
        print(result)
        
        # TODO: stop video recording via /stop_record (might be service|action)
        rospy.loginfo("Recording stopped!..")

    def start(self):
        self.state = "running"
        self.process()

    def stop(self):
        pass

    def checkStopFlags(self):
        pass


class PhotoTimelapse(Mission):
    
    def __init__(self, data):
        super().__init__()

        self.record_time = float(data.get('record_time', 100.0))
        self.play_time = float(data.get('play_time', 10.0))
        self.fps = int(data.get('fps', 25))
        self.degree = float(data.get('degree', 90.0))
        self.direction = bool(data.get('direction', False))
        self.no_of_photos1 = int(data.get('no_of_photos', 10))
        self.interval_time1 = float(data.get('interval_time', 10.0))

        self.no_of_photos = int(self.fps * self.play_time)
        self.interval_time = self.record_time / self.no_of_photos
        self.interval_degree = self.degree / self.no_of_photos

        rospy.loginfo(rospy.get_caller_id() + ": {} photo will be taken at each {} degree by {} sec".format(self.no_of_photos, self.interval_degree, self.interval_time))

        self.process_counter = 0

        self.state = "ready"

    def process(self):
        i = self.process_counter
        if i >= self.no_of_photos or self.checkStopFlags():
            return

        rospy.loginfo(rospy.get_caller_id() + ': image{0:04d}.jpg'.format(i))
        # TODO: trigger dslr to capture photo via /take_shot

        data = json.dumps({
            'command': 'rotate', 
            'degree': self.interval_degree, 
            'direction': self.direction
        })
        motor_cmd_msg = StringTriggerRequest(data=data)
        result = self.motor_cmd_service(motor_cmd_msg)
        rospy.loginfo(rospy.get_caller_id() + ": Result = {}, {}".format(result.success, result.message))

        self.process_counter += 1

    def start(self):
        self.state = "running"
        start_time = time.time()
        for i in range(self.no_of_photos):
            self.process()
            time.sleep(self.interval_time)
        
        elapsed_time = time.time() - start_time
        rospy.loginfo(rospy.get_caller_id() + ": Elapsed Time: {}".format(elapsed_time))
        self.state = "finished"

    def stop(self):
        self.state = "stopped"

    def checkStopFlags(self):
        return self.state == "stopped" or self.state == "uninitalized"
    

"""
def convert():
    system('convert -delay 10 -loop 0 image*.jpg animation.gif')
"""