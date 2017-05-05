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
import time
import random
from deap_lib import base
from deap_lib import creator
from deap_lib import tools

import sys

# Attribute generator
#                      define 'attr_bool' to be an attribute ('gene')
#                      which corresponds to integers sampled uniformly
#                      from the range [0,1] (i.e. 0 or 1 with equal
#                      probability)
# toolbox.register("attr_bool", random.randint, 0, 360)

# Structure initializers
#                         define 'individual' to be an individual
#                         consisting of 3 elements ('genes')
ATTR_LEGS_NUM_MIN, ATTR_LEGS_NUM_MAX = 4, 40
ATTR_ANGLE_BETWEEN_LEGS_MIN, ATTR_ANGLE_BETWEEN_LEGS_MAX = 0, 360
ATTR_OFFSET_BETWEEN_LEG_WAVES_MIN, ATTR_OFFSET_BETWEEN_LEG_WAVES_MAX = 0, 360
N_CYCLES = 1
POPULATION_SIZE = 5
# CXPB  is the probability with which two individuals
#       are crossed
#
# MUTPB is the probability for mutating an individual
#
# NGEN  is the number of generations for which the
#       evolution runs
CXPB = 0.5
MUTPB = 0.2
# IF YOU WANT CHANGE THE NUMBER OF GENERATIONS!
NGEN = 5


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

    return data


def world_generation():
    number_of_worlds = "number_of_worlds:=\"" + str(args['number_of_worlds']) + "\" "
    cage_height_range_begin = "cage_height_range_begin:=\"0.1\" "
    cage_height_range_end = "cage_height_range_end:=\"1.5\" "
    cage_width_and_lengh = "cage_width_and_lengh:=\"1.0\""
    all_args = number_of_worlds + cage_height_range_begin + cage_height_range_end + cage_width_and_lengh

    os.system("roslaunch strirus_ga_body_optimization full_world_generation.launch " + all_args)


def get_avg_dist_from_robot(legs_num, angle_between_legs, offset_between_leg_waves):
    all_data_from_cur_robot = []
    p = Pool()

    print("Number of legs and so on: " + str(legs_num) + " " + str(angle_between_legs) + " " + str(offset_between_leg_waves))
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

    return avg_dist


# the goal ('fitness') function to be maximized
# TODO implement
def fitness_function(individual):
    distance = individual[3]
    num_of_legs = individual[0]
    angle_btw_legs = individual[1]

    return distance / ((num_of_legs - 1) * math.sin(math.radians(angle_btw_legs))),

def crossover_function(ind1, ind2):
    size = len(ind1) - 1
    cxpoint1 = random.randint(1, size)
    cxpoint2 = random.randint(1, size - 1)
    if cxpoint2 >= cxpoint1:
        cxpoint2 += 1
    else:  # Swap the two cx points
        cxpoint1, cxpoint2 = cxpoint2, cxpoint1

    ind1[cxpoint1:cxpoint2], ind2[cxpoint1:cxpoint2] \
        = ind2[cxpoint1:cxpoint2], ind1[cxpoint1:cxpoint2]

    return ind1, ind2

def mutation_function(individual, indpb):
    if random.random() < indpb:
        individual[0] = random.randrange(4, 40)
    if random.random() < indpb:
        individual[1] = random.randrange(0, 360)
    if random.random() < indpb:
        individual[2] = random.randrange(0, 360)

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
        'simulation_time': '10'
    }
    args = updateArgs(args_default)
    # Generate world
    world_generation()
    # minimazing number of legs
    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)

    toolbox = base.Toolbox()

    toolbox.register("attr_legs_num", random.randint, ATTR_LEGS_NUM_MIN, ATTR_LEGS_NUM_MAX)
    toolbox.register("attr_angle_between_legs", random.randint, ATTR_ANGLE_BETWEEN_LEGS_MIN,
                     ATTR_ANGLE_BETWEEN_LEGS_MAX)
    toolbox.register("attr_offset_between_leg_waves", random.randint, ATTR_OFFSET_BETWEEN_LEG_WAVES_MIN,
                     ATTR_OFFSET_BETWEEN_LEG_WAVES_MAX)
    toolbox.register("attr_avg_dist", random.uniform, -sys.maxsize, sys.maxsize)
    toolbox.register("individual", tools.initCycle, creator.Individual,
                     (toolbox.attr_legs_num, toolbox.attr_angle_between_legs, toolbox.attr_offset_between_leg_waves,
                      toolbox.attr_avg_dist), n=N_CYCLES)

    # define the population to be a list of individuals
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # ----------
    # Operator registration
    # ----------
    # register the goal / fitness function
    toolbox.register("evaluate", fitness_function)

    # register the crossover operator
    toolbox.register("mate", crossover_function)

    # register a mutation operator with a probability to
    # flip each attribute/gene of 0.05
    toolbox.register("mutate", mutation_function, indpb=0.05)

    # operator for selecting individuals for breeding the next
    # generation: each individual of the current generation
    # is replaced by the 'fittest' (best) of three individuals
    # drawn randomly from the current generation.
    toolbox.register("select", tools.selTournament, tournsize=3)

    # MAIN FUNCTION
    random.seed()

    # create an initial population of 10 individuals (where
    # each individual is a list of integers)
    pop = toolbox.population(n=POPULATION_SIZE)

    print("Start of evolution")

    # Evaluate the entire population

    # FISRT EVOLUTION
    # fitnesses = list(map(toolbox.evaluate, pop))
    # for ind, fit in zip(pop, fitnesses):
    #    ind.fitness.values = fit

    print("  Evaluated %i individuals" % len(pop))

    # Begin the evolution
    for g in range(NGEN):
        print("-- Generation %i --" % g)

        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # HERE GAZEBO CODE
        for cur_robot in range(POPULATION_SIZE):
            print(str(cur_robot) + " individual try to cross through over the terrains")
            offspring[cur_robot][3] = get_avg_dist_from_robot(offspring[cur_robot][0], offspring[cur_robot][1],
                                                              offspring[cur_robot][2])
            print("Leave func with value: " + str(offspring[cur_robot][3]))
            time.sleep(10)

        # and analysing the output

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):

            # cross two individuals with probability CXPB
            if random.random() < CXPB:
                toolbox.mate(child1, child2)

                # fitness values of the children
                # must be recalculated later
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:

            # mutate an individual with probability MUTPB
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        print("  Evaluated %i individuals" % len(invalid_ind))

        # The population is entirely replaced by the offspring
        pop[:] = offspring

        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]

        # length = len(pop)
        # mean = sum(fits) / length
        # sum2 = sum(x * x for x in fits)
        # std = abs(sum2 / length - mean ** 2) ** 0.5

        # print("  Min %s" % min(fits))
        print("  Max %s" % max(fits))
        # print("  Avg %s" % mean)
        # print("  Std %s" % std)

    print("-- End of (successful) evolution --")

    best_ind = tools.selBest(pop, 1)[0]
    print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))

    # delete generated worlds
    delete_worlds()
