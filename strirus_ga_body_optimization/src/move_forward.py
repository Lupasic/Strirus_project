#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('move_forward')
    msg = Twist()
    msg.linear.x = 0.4
    while not rospy.is_shutdown():
        pub_vel.publish(msg)
        time.sleep(0.1)
