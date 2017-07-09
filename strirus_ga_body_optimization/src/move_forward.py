#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('move_forward')
velocity = rospy.get_param("~velocity")
msg = Twist()
msg.linear.x = velocity
while not rospy.is_shutdown():
    pub.publish(msg)
    time.sleep(0.1)
