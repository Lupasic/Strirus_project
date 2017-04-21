#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def talker():
    pub_left = []
    pub_right = []
    for num in range(19):
        pub_left.append(rospy.Publisher("strirus/leg_left_"+str(num)+"_revolute_controller/command", Float64, queue_size=10))
    for num in range(19):
        pub_right.append(rospy.Publisher("strirus/leg_right_"+str(num)+"_revolute_controller/command", Float64, queue_size=10))
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        for num in range(19):
            pub_left[num].publish(1)
        for num in range(19):
            pub_right[num].publish(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

