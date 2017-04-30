#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('move_forward')
r = rospy.Rate(10)
msg = Twist()
msg.linear.x = 4.0
while not rospy.is_shutdown():
   pub.publish(msg)
   r.sleep()















