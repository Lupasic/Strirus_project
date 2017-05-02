#!/usr/bin/env python
import random
import rospy
from os import mkdir
from os import path


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


def cage_height(param):
    if (param == "rand"):
        return random.uniform(args['cage_height_range_begin'], args['cage_height_range_end'])


def generate_terrain(index):
    if args['cell_width_number'] % 2 == 1:
        first_point = - ((args['cell_width_number'] - 1) / 2 * args['cage_width_and_lengh'])
    else:
        first_point = - (
            (args['cage_width_and_lengh'] / 2) + ((args['cell_width_number'] / 2 - 1) * args['cage_width_and_lengh']))

    temp = args['terrain_file_path_without_file_name'].split("/")
    temp = temp[-1]

    if not path.exists(args['terrain_file_path_without_file_name'][:-(len(temp) + 1)]):
        mkdir(args['terrain_file_path_without_file_name'][:-(len(temp) + 1)])
    if not path.exists(args['terrain_file_path_without_file_name'] + "_" + str(index)):
        mkdir(args['terrain_file_path_without_file_name'] + "_" + str(index))
    writeFile = open(args['terrain_file_path_without_file_name'] + "_" + str(index) + "/model.sdf", 'w')
    writeFile.write(
        "<?xml version='1.0'?>\n<sdf version='1.6'>\n	<model name='Terrain'>\n		<static>true</static>")

    lis = ["collision", "visual"]
    for i in range(args['cell_width_number']):
        for j in range(args['cell_length_number']):
            cur_cage_height = cage_height("rand")
            writeFile.write("\n		<link name=\"box_" + str(i) + "_" + str(j) + "\">\n")
            writeFile.write("			<pose>" + str(first_point + i * args['cage_width_and_lengh']) + " " + str(
                - j * args['cage_width_and_lengh']) + " " + str(cur_cage_height / 2) + " 0 0 0</pose>\n")
            writeFile.write(
                "			<inertial>\n				<mass>1.0</mass>\n				<inertia>\n					<ixx>0.083</ixx>\n 					<ixy>0.0</ixy>\n   					<ixz>0.0</ixz>\n   					<iyy>0.083</iyy>\n 					<iyz>0.0</iyz>\n   					<izz>0.083</izz>\n 				</inertia>\n			</inertial>\n")
            for nam in lis:
                writeFile.write("			<" + nam + " name=\"" + nam + "\">\n")
                writeFile.write("				<geometry>\n					<box>\n						")
                writeFile.write(
                    "<size>" + str(args['cage_width_and_lengh']) + " " + str(args['cage_width_and_lengh']) + " " + str(
                        cur_cage_height) + "</size>\n					</box>\n				</geometry>\n")
                writeFile.write("			</" + nam + ">\n")
            writeFile.write("		</link>\n")

    writeFile.write("	</model>\n</sdf>")
    writeFile.close()

    # also it is needed to generate model.sdf
    writeFile = open(args['terrain_file_path_without_file_name'] + "_" + str(index) + "/model.config", 'w')
    writeFile.write("<?xml version=\"1.0\" ?>\n<model>\n    <name>Terrain</name>\n    <version>1.0</version>\n    <sdf version=\"1.6\">model.sdf</sdf>\n<author>\n        <name>Bulichev Oleg</name>\n        <email>obulichev@yandex.ru</email>\n    </author>\n    <description>Terrain for testing robot's passability</description>\n</model>")
    writeFile.close()


def generate_world(index):
    temp = args['world_file_path_without_extention'].split("/")
    temp = temp[-1]
    if not path.exists(args['world_file_path_without_extention'][:-(len(temp) + 1)]):
        mkdir(args['world_file_path_without_extention'][:-(len(temp) + 1)])
    writeFile = open(args['world_file_path_without_extention'] + "_" + str(index) + ".world", 'w')
    writeFile.write(
        "<?xml version=\"1.0\" ?>\n<sdf version=\"1.6\">\n  <world name=\"default\">\n  <gravity>0 0 -9.81</gravity>\n")
    writeFile.write("  <physics name=\"ode_" + str(args[
                                                       'physics_iter']) + "iters\" type=\"ode\" default=\"true\">\n    <!-- the combination of 2 params below provides real time factor speed for simulation -->\n    <!-- max_real_time_factor = real_time_update_rate * max_step_size -->\n    <!-- min_real_time_factor depends only from max_step_size -->\n")
    writeFile.write("    <real_time_update_rate>" + str(
        args['real_time_update_rate']) + "</real_time_update_rate>\n    <max_step_size>" + str(args[
                                                                                                   'max_step_size']) + " </max_step_size>\n      <ode>\n        <solver>\n          <type>quick</type>\n          <iters>" + str(
        args['physics_iter']) + "</iters>\n        </solver>\n      </ode>\n    </physics>\n")

    writeFile.write(
        "      <include>\n      <uri>model://ground_plane</uri>\n    </include>\n    <include>\n      <uri>model://sun</uri>\n    </include>\n")
    writeFile.write("        <include>\n		<uri>model://" + str(args['package_name']) + "/" + str(
        args['terrain_path']) +"_"+str(index)+ "</uri>\n      <name>Terrain</name>\n      <pose>0 -2.0 0 0 0 0</pose>\n    </include>\n")

    writeFile.write("  </world>\n</sdf>")
    writeFile.close()


nodename = "full_world_generation"
node = rospy.init_node(nodename)
args_default = {
    'number_of_worlds': '2',
    'cage_height_range_begin': '0.1',
    'cage_height_range_end': '0.6',
    'cage_width_and_lengh': '1',
    'cell_width_number': '10',
    'cell_length_number': '10',
    'cage_height_param': 'rand',
    'terrain_file_path_without_file_name': '../maps/Generated_terrain',
    'world_file_path_without_extention': '../worlds/Generated_terrain/testing_area',
    'physics_iter': '400',
    'real_time_update_rate': '111.2',
    'max_step_size': '0.009',
    'package_name': 'strirus_ga_body_optimization',
    'terrain_path': '/maps/Generated_terrain/model.sdf'

}

args = updateArgs(args_default)
for index in range(args['number_of_worlds']):
    generate_terrain(index)
    generate_world(index)
