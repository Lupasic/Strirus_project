#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates


class Clock_dist_listener:
    clock = 0
    last_point = 0

    def __init__(self):
        # subscribers
        rospy.Subscriber("/clock", Clock, self.callback_clock)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_distance)

    def callback_clock(self, data):
        self.clock = data.clock.secs

    def callback_distance(self, data):
        # Both robot and terrain need to be spawned
        if len(data.pose) < 3:
            pass
        else:
            self.last_point = data.pose[2].position


if __name__ == '__main__':
    try:
        rospy.init_node("clock_dist_listener")
    except rospy.exceptions.ROSInitExeption:
        print("It cannot be init\n")
    cur_listener = Clock_dist_listener()
    rospy.spin()
