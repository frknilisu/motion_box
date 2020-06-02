from os import system
import time

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import Empty, SetBool
from demo.msg import TimelapseMsg, CmdStepMsg
from missions import VideoTimelapse, PhotoTimelapse


def callback_timelapse(msg):
    fps = msg.fps
    record_duration = msg.record_duration
    video_length = msg.video_length
    degree = msg.degree
    direction = msg.direction
    PhotoTimelapse(pub_cmd_step).run(record_duration,
                                     video_length, fps, degree, direction)
    VideoTimelapse(pub_cmd_step).run(video_length, degree, direction)


"""
def fibonacci_client():
    import​ actionlib
    from​ raspi_ws.msg i​mport​ SimpleMoveAction, SimpleMoveGoal, SimpleMoveResult, SimpleMoveFeedback
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(
        '/moveRel', SimpleMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    # Sends the goal to the action server.
    client.send_goal(SimpleMoveGoal(degree=180),
                     active_cb=callback_active,
                     feedback_cb=callback_feedback,
                     done_cb=callback_done)

    rospy.loginfo("Goal has been sent to the action server.")


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" %
                  (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    state = client.get_result()  # A SimpleMoveResult
    # Prints out the result of executing the action
    print("[Result] State: %d" % state)
    return state
"""


if __name__ == "__main__":
    try:
        rospy.init_node('missionController', log_level=rospy.DEBUG)
        rate = rospy.Rate(​0.5​)

        pub_cmd_step = rospy.Publish("/cmd_step", CmdStepMsg, queue_size=10)

        sub_timelapse = rospy.Subscriber(
            "/timelapse", TimelapseMsg, callback_timelapse)
        sub_simplemove = rospy.Subscriber(
            "/simple_move", CmdStepMsg, lambda msg: pub_cmd_step.publish(msg))

        while not​ rospy.is_shutdown():
            rospy.logdebug(​"rospy loop")​

            rate.sleep()
            # rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
