#!/usr/bin/env python

import rospy
import os
import math
import gc
import time
import logging
import rospy.exceptions
from clock_dist_listener import Clock_dist_listener
import subprocess
import signal
import random
from deap import base
from deap import creator
from deap import tools
from deap import algorithms
import numpy


robo_index = 0


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

    namespace = "individual" + str(index)
    ros_namespace = "namespace:=" + namespace + " "
    real_number_of_legs = "real_number_of_legs:=\"" + str(legs_num) + "\" "
    angle_between_legs = "angle_between_legs:=\"" + str(angle_between_legs) + "\" "
    offset_between_legs_waves = "offset_between_legs_waves:=\"" + str(offset_between_leg_waves) + "\" "
    cur_index = "cur_index:=\"" + str(index) + "\""

    roslaunch = subprocess.Popen(
        ['roslaunch', 'strirus_ga_body_optimization',
         'strirus_gazebo_with_auto_move_forward.launch', ros_namespace, real_number_of_legs, angle_between_legs,
         offset_between_legs_waves, cur_index])

    cur_listener = Clock_dist_listener(namespace)
    # subscribers
    rospy.logdebug('The PID of child: %d', roslaunch.pid)

    while True:
        if cur_listener.clock > args['simulation_time']:
            try:
                data['distance'] = math.sqrt(
                    math.pow(cur_listener.last_point.x, 2) + math.pow(cur_listener.last_point.y, 2) + math.pow(
                        cur_listener.last_point.z, 2))
                break
            except AttributeError:
                data['distance'] = 0
                rospy.logwarn("cur_listener is empty, distance = 0")
                break
    del cur_listener
    rospy.loginfo("Distance:= %f for %d terrain" % (data['distance'], index))
    roslaunch.send_signal(signal.SIGINT)
    # for avoidng zombie processes
    roslaunch.wait()
    del roslaunch

    return data


def world_generation():
    number_of_worlds = "number_of_worlds:=\"" + str(args['number_of_worlds']) + "\" "
    all_args = number_of_worlds
    os.system("roslaunch strirus_ga_body_optimization whole_worlds_generation.launch " + all_args)


def get_avg_dist_from_robot(legs_num, angle_between_legs, offset_between_leg_waves):
    all_data_from_cur_robot = []
    global robo_index
    rospy.loginfo("Robot_number:= %d has num_of_legs:= %d , angle_between_legs:= %d , offset_between_leg_waves:= %d",
                  robo_index % args['population_size'], legs_num, angle_between_legs, offset_between_leg_waves)
    robo_index += 1
    for i in range(int(args['number_of_worlds'])):
        rospy.loginfo('Terrain number:= %d', i)
        result = get_avg_dist_and_vel(i, legs_num, angle_between_legs, offset_between_leg_waves)
        all_data_from_cur_robot += [result]
    temp_sum = 0
    for temp in all_data_from_cur_robot:
        temp_sum = temp_sum + temp['distance']
    avg_dist = temp_sum / len(all_data_from_cur_robot)
    gc.collect()
    return avg_dist


# the goal ('fitness') function to be maximized
def fitness_function(individual):
    distance = get_avg_dist_from_robot(individual[0], individual[1], individual[2])
    num_of_legs = individual[0]
    angle_btw_legs = individual[1]
    length = ((num_of_legs - 1) * math.sin(math.radians(angle_btw_legs)))
    res = distance / length
    rospy.loginfo("AVG_dist is:= %f , length is %f , and the result is %f", distance, length, res)
    return res,


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
        'number_of_worlds': '4',
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
        'results_path': '',
        'logging_path': ''
    }

    args = updateArgs(args_default)
    # Generate world
    world_generation()
    # activate logging
    rospy.init_node("ga_body_optimization")
    temp = args['logging_path'].split("/")
    temp = temp[-1]
    if not os.path.exists(args['logging_path'][:-(len(temp) + 1)]):
        os.mkdir(args['logging_path'][:-(len(temp) + 1)])
    logging.basicConfig(format=u'%(levelname)-3s [%(asctime)s] %(message)s', level=logging.INFO,
                        filename=args['logging_path'])

    rospy.loginfo("START program")

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
    write_in_file([hof])
    print(hof)
    rospy.loginfo("FINISH program")
    rospy.signal_shutdown("finish program")
    # delete generated worlds
    delete_worlds()
