#!/usr/bin/env python

import rospy.impl.init
import os
import math
from multiprocessing import Pool
from functools import partial
import rospy
import rospy.exceptions
from Listener_class import Listener
import subprocess
import signal


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


def get_avg_dist_and_vel(index, legs_num, angle_between_legs, offset_between_leg_waves):
    data = {}
    cur_gazebo_port = args['gazebo_first_port'] + index
    cur_ros_port = args['rosmaster_first_port'] + index
    real_number_of_legs = "real_number_of_legs:=\"" + str(legs_num) + "\" "
    angle_between_legs = "angle_between_legs:=\"" + str(angle_between_legs) + "\" "
    offset_between_legs_waves = "offset_between_legs_waves:=\"" + str(offset_between_leg_waves) + "\" "
    cur_index = "cur_index:=\"" + str(index) + "\""

    neended_env = os.environ
    neended_env['GAZEBO_MASTER_URI'] = "http://localhost:" + str(cur_gazebo_port)
    roslaunch = subprocess.Popen(
        ['roslaunch', '-p', str(cur_ros_port), 'strirus_ga_body_optimization',
         'strirus_gazebo_with_auto_move_forward.launch', real_number_of_legs, angle_between_legs,
         offset_between_legs_waves, cur_index], env=neended_env)
    #change ROS_MASTER_URI for Listener node
    os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(cur_ros_port)

    # subscribers
    try:
        rospy.init_node("Listener")
    except rospy.exceptions.ROSInitExeption:
        print("It cannot be init\n")

    cur_listener = Listener()
    while not rospy.is_shutdown():
        if Listener.clock > args['simulation_time']:
            data['distance'] = math.sqrt(
                math.pow(cur_listener.last_point.x, 2) + math.pow(cur_listener.last_point.y, 2) + math.pow(
                    cur_listener.last_point.z, 2))
            data['velocity_avg'] = data['distance'] / cur_listener.clock
            rospy.signal_shutdown("Sim time is out")

    os.kill(roslaunch.pid, signal.SIGINT)

    return data


def world_generation():
    number_of_worlds = "number_of_worlds:=\"" + str(args['number_of_worlds']) + "\" "
    cage_height_range_begin = "cage_height_range_begin:=\"0.1\" "
    cage_height_range_end = "cage_height_range_end:=\"1.5\" "
    cage_width_and_lengh = "cage_width_and_lengh:=\"1.0\""
    all_args = number_of_worlds + cage_height_range_begin + cage_height_range_end + cage_width_and_lengh

    os.system("roslaunch strirus_ga_body_optimization full_world_generation.launch " + all_args)


def get_avg_vel_dist_from_robot(legs_num, angle_between_legs, offset_between_leg_waves):
    all_data_from_cur_robot = []
    p = Pool()

    for i in range(int(args['number_of_worlds'] / args['max_simultaneous_processes'])):
        index_arr = range(i * args['max_simultaneous_processes'],
                          args['max_simultaneous_processes'] + i * args['max_simultaneous_processes'])
        result = p.map(partial(get_avg_dist_and_vel, legs_num=legs_num, angle_between_legs=angle_between_legs,
                               offset_between_leg_waves=offset_between_leg_waves), index_arr)
        all_data_from_cur_robot = all_data_from_cur_robot + result
    return all_data_from_cur_robot


def delete_worlds():
    # delete all world files after working
    os.system("rm -r " + args['terrain_file_path_without_file_name'] + "_*")
    os.system("rm " + args['world_file_path_without_extention'] + "_*")


if __name__ == '__main__':
    args_default = {
        'terrain_file_path_without_file_name': '../maps/Generated_terrain',
        'world_file_path_without_extention': '../worlds/Generated_terrain/testing_area',
        'rosmaster_first_port': '1234',
        'gazebo_first_port': '11345',
        'number_of_worlds': '4',
        'max_simultaneous_processes': '2',
        'simulation_time': '10'
    }
    args = updateArgs(args_default)
    # Generate world
    world_generation()
    # get avg velosity and distance for robot with params
    data_from_robot = get_avg_vel_dist_from_robot(12, 60, 0)
    print(data_from_robot)
    data_from_robot = get_avg_vel_dist_from_robot(4, 90, 90)
    print(data_from_robot)
    # delete generated worlds
    delete_worlds()
