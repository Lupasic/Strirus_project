#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('move_forward')
r = rospy.Rate(30)
msg = Twist()
msg.linear.x = 0.4
while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

