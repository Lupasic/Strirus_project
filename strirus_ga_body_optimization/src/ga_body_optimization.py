#!/usr/bin/env python

import roslib

import rospy
#import rospy.impl.init
import os
import math
from multiprocessing import Pool
from functools import partial

import gc

import rospy.exceptions
from Listener_class import Listener
import subprocess
import signal
import time
import random
from deap import base
from deap import creator
from deap import tools
from deap import algorithms
import numpy



def write_in_file(array):
    temp = args['results_path'].split("/")
    temp = temp[-1]
    if not os.path.exists(args['results_path'][:-(len(temp) + 1)]):
        os.mkdir(args['results_path'][:-(len(temp) + 1)])
    writeFile = open(args['results_path'], 'a')

    str_array = []
    for i in array:
        str_array = str_array + [str(i)]
    writeFile.write(" ".join(str_array))
    writeFile.write("\n")
    writeFile.close()


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
    # change ROS_MASTER_URI for Listener node
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
            rospy.signal_shutdown("Sim time is out")

    os.kill(roslaunch.pid, signal.SIGINT)
    print("Exit from getting distance from terrain " + str(index))
    del cur_listener

    return data


def world_generation():
    number_of_worlds = "number_of_worlds:=\"" + str(args['number_of_worlds']) + "\" "
    all_args = number_of_worlds

    os.system("roslaunch strirus_ga_body_optimization whole_worlds_generation.launch " + all_args)


def get_avg_dist_from_robot(legs_num, angle_between_legs, offset_between_leg_waves):
    all_data_from_cur_robot = []
    p = Pool()

    print("Number of legs and so on: " + str(legs_num) + " " + str(angle_between_legs) + " " + str(
        offset_between_leg_waves))
    for i in range(int(args['number_of_worlds'] / args['max_simultaneous_processes'])):
        index_arr = range(i * args['max_simultaneous_processes'],
                          args['max_simultaneous_processes'] + i * args['max_simultaneous_processes'])
        result = p.map(partial(get_avg_dist_and_vel, legs_num=legs_num, angle_between_legs=angle_between_legs,
                               offset_between_leg_waves=offset_between_leg_waves), index_arr)
        all_data_from_cur_robot = all_data_from_cur_robot + result

    temp_sum = 0
    for temp in all_data_from_cur_robot:
        temp_sum = temp_sum + temp['distance']
    avg_dist = temp_sum / len(all_data_from_cur_robot)
    time.sleep(5)
    gc.collect()

    return avg_dist


# the goal ('fitness') function to be maximized
def fitness_function(individual):
    distance = get_avg_dist_from_robot(individual[0], individual[1], individual[2])
    num_of_legs = individual[0]
    angle_btw_legs = individual[1]
    print("distance in fitness func def: " + str(distance))
    print("length in fitness func def: " + str(((num_of_legs - 1) * math.sin(math.radians(angle_btw_legs)))))
    res = distance / ((num_of_legs - 1) * math.sin(math.radians(angle_btw_legs)))
    print(res)
    return distance / ((num_of_legs - 1) * math.sin(math.radians(angle_btw_legs))),


def mutation_function(individual, mutpb):
    if random.random() < mutpb:
        individual[0] = random.randrange(args['legs_num_min'], args['legs_num_max'])
    if random.random() < mutpb:
        individual[1] = random.randrange(args['angle_between_legs_min'], args['angle_between_legs_max'])
    if random.random() < mutpb:
        individual[2] = random.randrange(args['offset_between_leg_waves_min'], args['offset_between_leg_waves_max'])

    return individual,


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
        'simulation_time': '10',
        'number_generations': '',
        'population_size': '',
        'tournament_size': '',
        'crossover_probability': '',
        'mutation_probability': '',
        'legs_num_min': '',
        'legs_num_max': '',
        'angle_between_legs_min': '',
        'angle_between_legs_max': '',
        'offset_between_leg_waves_min': '',
        'offset_between_leg_waves_max': '',
        'results_path': ''
    }
    args = updateArgs(args_default)
    # Generate world
    world_generation()
    # minimazing number of legs
    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)

    toolbox = base.Toolbox()

    toolbox.register("attr_legs_num", random.randint, args['legs_num_min'], args['legs_num_max'])
    toolbox.register("attr_angle_between_legs", random.randint, args['angle_between_legs_min'],
                     args['angle_between_legs_max'])
    toolbox.register("attr_offset_between_leg_waves", random.randint, args['offset_between_leg_waves_min'],
                     args['offset_between_leg_waves_max'])

    toolbox.register("individual", tools.initCycle, creator.Individual,
                     (toolbox.attr_legs_num, toolbox.attr_angle_between_legs, toolbox.attr_offset_between_leg_waves),
                     n=1)

    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("evaluate", fitness_function)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", mutation_function, mutpb=args['mutation_probability'])
    toolbox.register("select", tools.selTournament, tournsize=args['tournament_size'])

    pop = toolbox.population(n=args['population_size'])
    hof = tools.HallOfFame(1)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", numpy.mean)
    stats.register("std", numpy.std)
    stats.register("min", numpy.min)
    stats.register("max", numpy.max)

    pop, log = algorithms.eaSimple(pop, toolbox, cxpb=args['crossover_probability'], mutpb=args['mutation_probability'],
                                   ngen=args['number_generations'],
                                   stats=stats, halloffame=hof, verbose=True)

    write_in_file([log])
    print(hof)
    # delete generated worlds
    delete_worlds()
