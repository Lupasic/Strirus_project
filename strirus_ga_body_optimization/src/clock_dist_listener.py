#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates


class ClockDistListener:
    '''
    Class, which subscribe on clock and distance and storage this info in itself
    '''

    def __init__(self, namespace=None):
        '''
        Subscribe on clock and distance topics
        :param namespace: provide remapping on other topics
        :type namespace: str
        '''
        # subscribers
        self.clock = 0
        self.last_point = None
        self.namespace = namespace
        if namespace == None or namespace == "/":
            rospy.Subscriber("/clock", Clock, self.callback_clock)
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_distance)
            # just for avoid error messages
            pub = rospy.Publisher(self.namespace + "/clock", Clock, queue_size=10)
            pub.publish(0)
        else:
            rospy.Subscriber(self.namespace + "/clock", Clock, self.callback_clock)
            rospy.Subscriber(self.namespace + "/gazebo/model_states", ModelStates, self.callback_distance)

    def callback_clock(self, data):
        self.clock = data.clock.secs

    def callback_distance(self, data):
        # Both robot and terrain need to be spawned
        if len(data.pose) < 3:
            pass
        else:
            self.last_point = data.pose[2].position

    def set_clock(self, time):
        self.clock = time
