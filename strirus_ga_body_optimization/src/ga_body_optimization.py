#!/usr/bin/env python

import rospy
import os
import math
import gc
from multiprocessing import Pool
from functools import partial
from logger import Logger
import rospy.exceptions
from clock_dist_listener import ClockDistListener
import subprocess
import signal
import time
import random
from deap import base
from deap import creator
from deap import tools
from deap import algorithms
import numpy

_robo_index = 0


def write_in_file(array):
    '''
    Write results in file, results_path can specify in roslaunch file (body_optimization.launch)

    :param array: array of phrases
    :type array: list
    '''
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


def dist_log(dist):
    '''
    Func for write distance from each simulation. Can be turn off in roslaunch file (body_optimization.launch)

    :param dist: distance
    :type dist: float
    '''
    temp = args['dist_path'].split("/")
    temp = temp[-1]
    if not os.path.exists(args['dist_path'][:-(len(temp) + 1)]):
        os.mkdir(args['dist_path'][:-(len(temp) + 1)])
    dist_file = open(args['dist_path'], 'a')
    # For finding first iteration
    if _robo_index == 0:
        dist_file.write("Start \n")

    dist_file.write(str(dist))
    # Value need to be matched manually
    if dist > 0.25:
        dist_file.write(" +")
    dist_file.write("\n")
    dist_file.close()


def update_args(arg_defaults):
    '''
    Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver

    :return: available args from roslaunch files
    :rtype: dict
    '''
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
    '''
    Start gazebo roslaunch file, which activate gazebo simulation. It will stop, when sim.time reach args['sim time'] (can be specified in roslaunch file(body_optimization.launch))

    :param index: world index (world, which will be launched)
    :param legs_num: number of legs from one side
    :type legs_num: int
    :param angle_between_legs: angle between neighbor legs
    :type angle_between_legs: int
    :param offset_between_leg_waves: offset between legs from opposite sides
    :type offset_between_leg_waves: int
    :return data: store several information about simulation results (at the moment, only distance)
    :raise AttributeError: sometimes gazebo crashed and data is vanish.
    '''
    data = {}
    cur_gazebo_port = args['gazebo_first_port'] + index
    cur_ros_port = args['rosmaster_first_port'] + index

    real_number_of_legs = "real_number_of_legs:=\"" + str(legs_num) + "\" "
    angle_between_legs = "angle_between_legs:=\"" + str(angle_between_legs) + "\" "
    offset_between_legs_waves = "offset_between_legs_waves:=\"" + str(offset_between_leg_waves) + "\" "
    cur_index = "cur_index:=\"" + str(index) + "\""

    neended_env = os.environ
    neended_env['GAZEBO_MASTER_URI'] = "http://lupasic-computer:" + str(cur_gazebo_port)

    roslaunch = subprocess.Popen(
        ['roslaunch', '-p', str(cur_ros_port), 'strirus_ga_body_optimization',
         'strirus_gazebo_with_auto_move_forward.launch', real_number_of_legs, angle_between_legs,
         offset_between_legs_waves, cur_index], env=neended_env)

    # change ROS_MASTER_URI for Listener node
    os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(cur_ros_port)

    rospy.logdebug('The PID of child: %d', roslaunch.pid)
    rospy.logdebug('Terrain number is:= %d', index)

    cur_listener = ClockDistListener("terrain")

    while True:
        if cur_listener.clock > args['simulation_time']:
            try:
                data['distance'] = math.sqrt(
                    math.pow(cur_listener.last_point.x, 2) + math.pow(cur_listener.last_point.y, 2) + math.pow(
                        cur_listener.last_point.z, 2))
                rospy.signal_shutdown("Sim time is out")
                break
            except AttributeError:
                data['distance'] = 0
                cur_logger.logWarn("cur_listener is empty, distance = 0")
                rospy.logwarn("cur_listener is empty, distance = 0")
                break

    rospy.loginfo("Distance:= %f for %d terrain" % (data['distance'], index))
    cur_logger.logInfo("Distance:= " + str(data['distance']) + " for " + str(index) + " terrain")

    if args['extra_dist_log'] == True:
        dist_log(data['distance'])

    # kill roslaunch. SIGINT is the only way, otherwise rosmaster will araise
    roslaunch.send_signal(signal.SIGINT)
    # for avoidng zombie processes
    roslaunch.wait()
    del roslaunch

    return data


def world_generation():
    '''
    Generate worlds, number of worlds can be specify in roslaunch (body_optimization.launch)
    '''
    number_of_worlds = "number_of_worlds:=\"" + str(args['number_of_worlds']) + "\" "
    all_args = number_of_worlds
    os.system("roslaunch strirus_ga_body_optimization whole_worlds_generation.launch " + all_args)


def get_avg_dist_from_robot(legs_num, angle_between_legs, offset_between_leg_waves):
    '''
    Launch the sequence of world simulations, number of worlds can be specify in roslaunch file (body_optimization.launch)

    :param legs_num: number of legs from one side
    :type legs_num: int
    :param angle_between_legs: angle between neighbor legs
    :type angle_between_legs: int
    :param offset_between_leg_waves: offset between legs from opposite sides
    :type offset_between_leg_waves: int
    :return: average distance from all worlds
    '''
    all_data_from_cur_robot = []
    global _robo_index

    cur_logger.logInfo("Robot_number:= " + str(_robo_index % args['population_size']) + " has num_of_legs:= " + str(
        legs_num) + " , angle_between_legs:= " + str(angle_between_legs) + " , offset_between_leg_waves:= " + str(
        offset_between_leg_waves))

    rospy.loginfo("Robot_number:= %d has num_of_legs:= %d , angle_between_legs:= %d , offset_between_leg_waves:= %d",
                  _robo_index % args['population_size'], legs_num, angle_between_legs, offset_between_leg_waves)
    _robo_index += 1

    for i in range(int(args['number_of_worlds'] / args['max_simultaneous_processes'])):
        p = Pool()
        index_arr = range(i * args['max_simultaneous_processes'],
                          args['max_simultaneous_processes'] + i * args['max_simultaneous_processes'])
        rospy.loginfo('Terrain range is:= %d - %d', index_arr[0], index_arr[-1])
        result = p.map(partial(get_avg_dist_and_vel, legs_num=legs_num, angle_between_legs=angle_between_legs,
                               offset_between_leg_waves=offset_between_leg_waves), index_arr)
        all_data_from_cur_robot = all_data_from_cur_robot + result
        p.terminate()
        del p
    temp_sum = 0
    for temp in all_data_from_cur_robot:
        temp_sum = temp_sum + temp['distance']
    avg_dist = temp_sum / len(all_data_from_cur_robot)
    gc.collect()
    return avg_dist


# the goal ('fitness') function to be maximized
def fitness_function(individual):
    '''
    Calculate fitness function - coeffs for distance and length can be specified in roslaunch file (body_optimization.launch)

    :param individual: data storage, which cosist number of legs [0], angle between legs [1] and offset between legs from opposite sides [2]
    :return: value of fitness function
    '''
    distance = get_avg_dist_from_robot(individual[0], individual[1], individual[2])
    num_of_legs = individual[0]
    angle_btw_legs = individual[1]
    length = (num_of_legs - 1) * math.sin(math.radians(angle_btw_legs))

    fitness = args['beta_coeff'] * (args['dist_coeff'] * distance + args['length_coeff'] * (1 / length)) + (1 - args[
'beta_coeff']) * (pow(distance, args['dist_coeff']) * pow((1 / length), args['length_coeff']))

    rospy.loginfo("AVG_dist is:= %f , length is %f , and the result is %f", distance, length, fitness)
    cur_logger.logInfo(
        "AVG_dist is:= " + str(distance) + " , length is " + str(length) + " , and the result is " + str(fitness))
    return fitness,


def mutation_function(individual, mutpb):
    '''
    Mutation function, which randomly change individual's values
    :param individual: individual: data storage, which cosist number of legs [0], angle between legs [1] and offset between legs from opposite sides [2]
    :param mutpb: percentage of success mutation
    :return:
    '''
    count_mut = 0
    max_mutation_iter = 2
    if random.random() < mutpb and count_mut < max_mutation_iter:
        individual[0] = random.randrange(args['legs_num_min'], args['legs_num_max'])
        count_mut += 1
    if random.random() < mutpb and count_mut < max_mutation_iter:
        individual[1] = random.randrange(args['angle_between_legs_min'], args['angle_between_legs_max'])
        count_mut += 1
    if random.random() < mutpb and count_mut < max_mutation_iter:
        individual[2] = random.randrange(args['offset_between_leg_waves_min'], args['offset_between_leg_waves_max'])
        count_mut += 1

    return individual,


def delete_worlds():
    '''
    Delete all world files after working
    '''
    os.system("rm -r " + args['terrain_file_path_without_file_name'] + "_*")
    os.system("rm " + args['world_file_path_without_extention'] + "_*")


if __name__ == '__main__':
    args_default = {
        'terrain_file_path_without_file_name': '../maps/Generated_terrain',
        'world_file_path_without_extention': '../worlds/Generated_terrain/testing_area',
        'rosmaster_first_port': '1234',
        'gazebo_first_port': '11345',
        'number_of_worlds': 4,
        'max_simultaneous_processes': 2,
        'simulation_time': 10,
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
        'results_path': '',
        'logging_path': '',
        'dist_coeff': 1,
        'length_coeff': 1,
        'generate_worlds': 'False',
        'dist_path': '',
        'extra_dist_log': 'True',
        'ga_repetition_num': 5,
        'beta_coeff': 1
    }

    args = update_args(args_default)
    # Generate world
    if args['generate_worlds'] == True or not os.path.exists(args['terrain_file_path_without_file_name'] + "_" + str(
                    args['number_of_worlds'] - 1) or os.path.exists(
                args['world_file_path_without_extention'] + ".world")):
        world_generation()

    # activate logging
    cur_logger = Logger(args['logging_path'])

    for i in range(args['ga_repetition_num']):
        cur_logger.logInfo("START program")
        rospy.loginfo("START program")

        creator.create("FitnessMax", base.Fitness, weights=(1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMax)

        toolbox = base.Toolbox()

        toolbox.register("attr_legs_num", random.randint, args['legs_num_min'], args['legs_num_max'])
        toolbox.register("attr_angle_between_legs", random.randint, args['angle_between_legs_min'],
                         args['angle_between_legs_max'])
        toolbox.register("attr_offset_between_leg_waves", random.randint, args['offset_between_leg_waves_min'],
                         args['offset_between_leg_waves_max'])

        toolbox.register("individual", tools.initCycle, creator.Individual,
                         (
                             toolbox.attr_legs_num, toolbox.attr_angle_between_legs,
                             toolbox.attr_offset_between_leg_waves),
                         n=1)

        toolbox.register("population", tools.initRepeat, list, toolbox.individual)

        toolbox.register("evaluate", fitness_function)
        toolbox.register("mate", tools.cxTwoPoint) #crossover
        toolbox.register("mutate", mutation_function, mutpb=args['mutation_probability'])
        toolbox.register("select", tools.selTournament, tournsize=args['tournament_size'])

        pop = toolbox.population(n=args['population_size'])
        hof = tools.HallOfFame(1)
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", numpy.mean)
        stats.register("std", numpy.std)
        stats.register("min", numpy.min)
        stats.register("max", numpy.max)

        pop, log = algorithms.eaSimple(pop, toolbox, cxpb=args['crossover_probability'],
                                       mutpb=args['mutation_probability'],
                                       ngen=args['number_generations'],
                                       stats=stats, halloffame=hof, verbose=True)

        write_in_file([log])
        write_in_file([hof])
        print(hof)
        rospy.loginfo("FINISH program")
        cur_logger.logInfo("FINISH program")
    del cur_logger
    rospy.signal_shutdown("finish program")

    # delete generated worlds
    if args['generate_worlds'] == True:
        delete_worlds()
