#!/usr/bin/env python

import rospy.impl.init
import os
import math
from multiprocessing import Pool
import rospy
import rospy.exceptions
from Listener_class import Listener

# need to divided without remainder
# TODO make it server params
WORLDS_NUM = 4
MAX_PROCCESSES = 1
ROS_FIRST_PORT = 1234
GAZEBO_FIRST_PORT = 11345
SIM_TIME_MAX = 10


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


def get_avg_dist_and_vel(index):
    data = {}
    cur_gazebo_port = GAZEBO_FIRST_PORT + index
    cur_ros_port = ROS_FIRST_PORT + index

    real_number_of_legs = "real_number_of_legs:=\"" + str(19) + "\" "
    angle_between_legs = "angle_between_legs:=\"60\" "
    offset_between_legs_waves = "offset_between_legs_waves:=\"0\" "
    cur_index = "cur_index:=\"" + str(index) + "\""
    all_args = real_number_of_legs + angle_between_legs + offset_between_legs_waves + cur_index
    print("cur index !!!!!!!!! " + cur_index)
    os.system("GAZEBO_MASTER_URI=http://localhost:" + str(cur_gazebo_port) + " roslaunch -p  " + str(
        cur_ros_port) + " strirus_ga_body_optimization strirus_gazebo_with_auto_move_forward.launch " + all_args + " &")
    os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(cur_ros_port)

    # subscribers
    try:
        rospy.init_node("Listener")
    except rospy.exceptions.ROSInitExeption:
        print("It cannot be init\n")

    cur_listener = Listener()
    while not rospy.is_shutdown():
        if Listener.clock > SIM_TIME_MAX:
            data['distance'] = math.sqrt(
                math.pow(cur_listener.last_point.x, 2) + math.pow(cur_listener.last_point.y, 2) + math.pow(
                    cur_listener.last_point.z, 2))
            data['velocity_avg'] = data['distance'] / cur_listener.clock
            rospy.signal_shutdown("Sim time is out")

    # It is killed roslaunch process, tail -n3 (roslaunch, command below, grep), head -n1 -> roslaunch pid
    os.system(
        "ps axu | grep \"roslaunch -p " + str(cur_ros_port) + "\" | cut -d ' ' -f3 | tail -n3 | head -n1 | xargs kill")

    return data


# Generate worlds
if __name__ == '__main__':
    args_default = {
        'terrain_file_path_without_file_name': '../maps/Generated_terrain',
        'world_file_path_without_extention': '../worlds/Generated_terrain/testing_area'
    }
    args = updateArgs(args_default)

    number_of_worlds = "number_of_worlds:=\"" + str(WORLDS_NUM) + "\" "
    cage_height_range_begin = "cage_height_range_begin:=\"0.1\" "
    cage_height_range_end = "cage_height_range_end:=\"1.5\" "
    cage_width_and_lengh = "cage_width_and_lengh:=\"1.0\""
    all_args = number_of_worlds + cage_height_range_begin + cage_height_range_end + cage_width_and_lengh

    print(all_args)

    os.system("roslaunch strirus_ga_body_optimization full_world_generation.launch " + all_args)

    all_data_from_cur_robot = []

    p = Pool()
    for i in range(int(WORLDS_NUM / MAX_PROCCESSES)):
        result = p.map(get_avg_dist_and_vel, range(i * MAX_PROCCESSES, MAX_PROCCESSES + i * MAX_PROCCESSES))
        all_data_from_cur_robot = all_data_from_cur_robot + result

    print(all_data_from_cur_robot)

    # TODO delete this output
    if not os.path.exists("/home/lupasic/Programs/dich.txt"):
        writeFile = open("/home/lupasic/Programs/dich.txt", "w")
    else:
        writeFile = open("/home/lupasic/Programs/dich.txt", "a")

    writeFile.write("\n" + str(all_data_from_cur_robot))
    writeFile.close()
    # delete all world files after working
    os.system("rm -r " + args['terrain_file_path_without_file_name'] + "_*")
    os.system("rm " + args['world_file_path_without_extention'] + "_*")
