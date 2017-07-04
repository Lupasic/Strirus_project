#!/usr/bin/env python
import random
import rospy


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


def cage_height(param, ex_args=None):
    if (param == "rand"):
        return random.uniform(args['cage_height_range_begin'], args['cage_height_range_end'])
    if (param == "each_cage_normalvariate"):
        return random.normalvariate((args['cage_height_range_end'] - args['cage_height_range_begin']) / 2,
                                    args['std_deviation'])
    if (param == "gauss_terrain"):
        return random.gauss(ex_args, args['std_deviation'])


nodename = "generating_terrain"
node = rospy.init_node(nodename)

args_default = {
    'cage_height_range_begin': '0.1',
    'cage_height_range_end': '0.6',
    'cage_width_and_lengh': '1',
    'cell_width_number': '10',
    'cell_length_number': '10',
    'cage_height_param': 'rand',
    'file_path': '../maps/Generated_terrain/model.sdf',
    'scale_coeff': '0',
    'std_deviation': '0.4',
    'two_dimension_terrain': '1'
}
args = updateArgs(args_default)

if args['cell_width_number'] % 2 == 1:
    first_point = - ((args['cell_width_number'] - 1) / 2 * args['cage_width_and_lengh'])
else:
    first_point = - (
        (args['cage_width_and_lengh'] / 2) + ((args['cell_width_number'] / 2 - 1) * args['cage_width_and_lengh']))

writeFile = open(args['file_path'], 'w')
writeFile.write("<?xml version='1.0'?>\n<sdf version='1.6'>\n	<model name='Terrain'>\n		<static>true</static>")

if args["cage_height_param"] == "gauss_terrain":
    scale = (args['cage_height_range_end'] - args['cage_height_range_begin']) / (
        (args['cell_length_number'] // 2) - args['scale_coeff'])
    temp = (args['cell_length_number'] // 2) - args['scale_coeff']
    scale_seq = []
    for i in range(args['cell_length_number'] // 2):
        if i > temp:
            scale_seq.append(temp)
        else:
            scale_seq.append(i)
    scale_seq += list(reversed(scale_seq))
    if (args['cell_length_number'] % 2 == 1):
        scale_seq.insert(len(scale_seq) / 2, temp)

lis = ["collision", "visual"]
for i in range(args['cell_length_number']):
    cur_cage_height = 0
    if args["two_dimension_terrain"] == True:
        if args['cage_height_param'] == "gauss_terrain":
            cur_cage_height = cage_height(args['cage_height_param'],
                                          ex_args=args['cage_height_range_begin'] + scale_seq[i] * scale)
        else:
            cur_cage_height = cage_height(args["cage_height_param"])

    for j in range(args['cell_width_number']):
        if args["two_dimension_terrain"] == False:
            if args['cage_height_param'] == "gauss_terrain":
                cur_cage_height = cage_height(args['cage_height_param'],
                                              ex_args=args['cage_height_range_begin'] + scale_seq[i] * scale)
            else:
                cur_cage_height = cage_height(args["cage_height_param"])
        writeFile.write("\n		<link name=\"box_" + str(i) + "_" + str(j) + "\">\n")
        writeFile.write("			<pose>" + str(first_point + j * args['cage_width_and_lengh']) + " " + str(
            -i * args['cage_width_and_lengh']) + " " + str(cur_cage_height / 2) + " 0 0 0</pose>\n")
        writeFile.write(
            "			<inertial>\n				<mass>1.0</mass>\n"
            "				<inertia>\n					<ixx>0.0</ixx>\n 					<ixy>0.0</ixy>\n"
            "   					<ixz>0.0</ixz>\n   					<iyy>0.0</iyy>\n 					<iyz>0.0</iyz>\n"
            "   					<izz>0.0</izz>\n 				</inertia>\n			</inertial>\n")
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
