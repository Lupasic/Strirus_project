#!/usr/bin/env python
import random
import os
import time
#from full_world_generation import delete_files
WORLDS_NUM = 3

#Generate worlds
number_of_worlds = "number_of_worlds:=\""+str(WORLDS_NUM)+"\" "
cage_height_range_begin = "cage_height_range_begin:=\"0.1\" "
cage_height_range_end = "cage_height_range_end:=\"1.5\" "
cage_width_and_lengh = "cage_width_and_lengh:=\"1.0\""
all_args = number_of_worlds + cage_height_range_begin + cage_height_range_end + cage_width_and_lengh

os.system("roslaunch strirus_ga_body_optimization full_world_generation.launch "+all_args)

time.sleep(5)

os.system("rm ../maps/Generated_terrain/model_*")
os.system("rm ../worlds/testing_area_*")
#delete_files()

