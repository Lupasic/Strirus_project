#!/usr/bin/env python

import os
import math
import time
from multiprocessing import Pool
import rospy
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates

#need to divided without remainder
WORLDS_NUM = 2
MAX_PROCCESSES = 2
ROS_FIRST_PORT = 1240
GAZEBO_FIRST_PORT = 11346
SIM_TIME_MAX = 30

def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver.'''
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)  # search without postfix
        if full_name is None:  # search with postfix
            full_name = rospy.search_param(name)
        if full_name is None:  # use default
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
    return (args)


class Listener:
    clock = 0
    last_point = 0

    def __init__(self):
        # subscribers
        listener = rospy.init_node('listeners')
        clock_subscriber = rospy.Subscriber("/clock", Clock, self.callback_clock)
        distance_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_distance)

    def callback_clock(data):
        Listener.clock = data.clock.secs

    def callback_distance(data):
        Listener.last_point = data.pose[2]


def get_avg_dist_and_vel(index):
    data = {}
    real_number_of_legs = "real_number_of_legs:=\"" + str(19) + "\" "
    angle_between_legs = "angle_between_legs:=\"60\" "
    offset_between_legs_waves = "offset_between_legs_waves:=\"0\" "
    cur_index = "cur_index:=\""+str(index)+"\""
    all_args = real_number_of_legs + angle_between_legs + offset_between_legs_waves + cur_index
    print("cur index !!!!!!!!! " + cur_index)
    os.system("GAZEBO_MASTER_URI=http://localhost:"+str(GAZEBO_FIRST_PORT+index)+" roslaunch -p  "+str(ROS_FIRST_PORT+index)+" strirus_ga_body_optimization strirus_gazebo_with_auto_move_forward.launch "+all_args+" &")
    os.environ['ROS_MASTER_URI'] = "http://localhost:"+str(ROS_FIRST_PORT+index)

    #subscribers
    # cur_listener = Listener()
    # while not rospy.is_shutdown():
    #     if Listener.clock > SIM_TIME_MAX:
    #         data['distance'] = math.sqrt(
    #             math.pow(cur_listener.last_point.x, 2) + math.pow(cur_listener.last_point.y, 2) + math.pow(
    #                 cur_listener.last_point.z, 2))
    #         data['velocity_avg'] = data['distance'] / cur_listener.clock
    #         rospy.signal_shutdown()
    #         os.system("kill %1")
    time.sleep(20)

    os.system("killall gzclient || killall gzserver")
    data = index

    return data

#Generate worlds
if __name__ == '__main__':
    nodename = "ga_body_optimization"
    node = rospy.init_node(nodename)
    args_default = {
        'terrain_file_path_without_file_name': '../maps/Generated_terrain',
        'world_file_path_without_extention': '../worlds/Generated_terrain/testing_area'
    }
    args = updateArgs(args_default)

    number_of_worlds = "number_of_worlds:=\""+str(WORLDS_NUM)+"\" "
    cage_height_range_begin = "cage_height_range_begin:=\"0.1\" "
    cage_height_range_end = "cage_height_range_end:=\"1.5\" "
    cage_width_and_lengh = "cage_width_and_lengh:=\"1.0\""
    all_args = number_of_worlds + cage_height_range_begin + cage_height_range_end + cage_width_and_lengh

    print(all_args)

    os.system("roslaunch strirus_ga_body_optimization full_world_generation.launch "+all_args)

    all_data_from_cur_robot = []

    p = Pool()
    for i in range(int(WORLDS_NUM / MAX_PROCCESSES)):
        result = p.map(get_avg_dist_and_vel, range(i * MAX_PROCCESSES, MAX_PROCCESSES + i * MAX_PROCCESSES))
        all_data_from_cur_robot = all_data_from_cur_robot + result

    print(all_data_from_cur_robot)

    #delete all world files after working
    os.system("rm -r " + args['terrain_file_path_without_file_name'] + "_*")
    os.system("rm " + args['world_file_path_without_extention'] + "_*")

