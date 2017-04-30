#!/usr/bin/env python
import random
#import rospy
import os
import threading

def ros(loc, rloc, dich):
    os.system("GAZEBO_MASTER_URI=http://localhost:"+ str(loc) +" roslaunch -p "+ str(rloc) +" strirus_ga_body_optimization strirus_gazebo_with_auto_move_forward.launch cage_width_and_lengh:=\""+ str(dich)+"\" &")
    os.environ['ROS_MASTER_URI'] = "http://lupasic-computer:"+ rloc +"/"



t1 = threading.Thread(target=ros, args=(11346, 1234, 2.0))
t2 = threading.Thread(target=ros, args=(11347, 1235, 1.0))

t1.start()
t2.start()

t1.join()
t2.join()


