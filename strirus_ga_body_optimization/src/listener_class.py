#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates
from multiprocessing.sharedctypes import Value
from ctypes import Structure, c_float


class Listener:
    clock = 0
    last_point = 0
    max_sim_processes = 1

    def __init__(self, namespace=None, max_sim_processes=1):
        # subscribers
        self.namespace = namespace
        self.max_sim_processes = max_sim_processes
        self.clock = [0] * self.max_sim_processes
        self.last_point = [0] * self.max_sim_processes
        for i in range(max_sim_processes):
            rospy.Subscriber("/" + self.namespace + str(i) + "/clock", Clock, self.callback_clock)
            rospy.Subscriber("/" + self.namespace + str(i) + "/gazebo/model_states", ModelStates,
                             self.callback_distance)

    def callback_clock(self, data):
        self.clock = data.clock.secs

    def callback_distance(self, data):
        # Both robot and terrain need to be spawned
        if len(data.pose) < 3:
            pass
        else:
            self.last_point = data.pose[2].position


if __name__ == '__main__':
    rospy.init_node("listener")
    cur_listener = Listener()
    rospy.spin()
