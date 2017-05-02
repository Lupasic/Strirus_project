import os
import rospy

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates


def callback_clock(data):
    Listener.clock = data.clock.secs


def callback_distance(data):
    if Listener.clock != 0:
        Listener.last_point = data.pose[2].position


class Listener:
    clock = 0
    last_point = 0

    def __init__(self):
        # subscribers
        rospy.Subscriber("/clock", Clock, callback_clock)
        rospy.Subscriber("/gazebo/model_states", ModelStates, callback_distance)
