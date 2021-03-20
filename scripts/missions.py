#!/usr/bin/python3

import rospy
import time

from std_msgs.msg import String

class VideoTimelapse:

    def __init__(self, pub):
        self.pub_motor_cmd = pub

    def run(self, mission_msg):
        video_length = mission_msg.get('video_length', 7200)
        degree = mission_msg.get('degree', 7200)
        direction = mission_msg.get('direction', 7200)

        rospy.loginfo("Recording started!..")
        # TODO: start video recording via /start_record (might be service|action)

        motor_cmd_msg = json.dumps({'command': 'rotate', 'degree': degree, 'direction': direction})
        self.pub_motor_cmd.publish(motor_cmd_msg)
        
        # TODO: stop video recording via /stop_record (might be service|action)
        rospy.loginfo("Recording stopped!..")


class PhotoTimelapse:
    
    def __init__(self, mission_msg):
        self._state = "uninitialized"

        self.record_duration = mission_msg.get('record_duration', 100.0)
        self.video_length = mission_msg.get('video_length', 10.0)
        self.fps = mission_msg.get('fps', 25)
        self.degree = mission_msg.get('degree', 10.0)
        self.direction = mission_msg.get('direction', "cw")

        self.no_of_photo = int(self.fps * self.video_length)
        self.interval_duration = self.record_duration / self.no_of_photo
        self.interval_degree = self.degree / self.no_of_photo

        rospy.loginfo("{} photo will be taken at each {} degree by {} sec".format(self.no_of_photo, self.interval_degree, self.interval_duration))

        rospy.wait_for_service('motor/cmd')
        self.motor_cmd_service = rospy.ServiceProxy('motor/cmd', String)

        self.state = "ready"

    def process(self, i):
        if i >= self.no_of_photo or checkStopFlags():
            return

        rospy.loginfo('image{0:04d}.jpg'.format(i))
        # TODO: trigger dslr to capture photo via /take_shot

        motor_cmd_msg = json.dumps({
            'command': 'rotate', 
            'degree': self.interval_degree, 
            'direction': self.direction
        })
        _ = self.motor_cmd_service(motor_cmd_msg)

    def start(self):
        self.state = "running"
        start_time = time.time()
        for i in range(self.no_of_photo):
            self.process(i)
            time.sleep(self.interval_duration)
        
        elapsed_time = time.time() - start_time
        rospy.loginfo("Elapsed Time: {}".format(elapsed_time))
        self.state = "finished"

    def stop(self):
        self.state = "stopped"

    def checkStopFlags(self):
        return self.state == "stopped" or self.state == "uninitalized"

    @property
	def state(self):
		return self._state
	
	@state.setter
	def state(self, state):
        self._state = state

	@state.deleter
	def state(self):
		del self._state

"""
def convert():
    system('convert -delay 10 -loop 0 image*.jpg animation.gif')
"""