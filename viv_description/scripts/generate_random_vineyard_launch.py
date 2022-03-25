#!/usr/bin/env python
"""
    Script for generating random vineyard from three plant models
    Run with:
        ./generate_random_vineyard_launch N_plants_in_row N_rows
"""
import math
import random
import sys

def spawn_model(current_n, current_grapevine_model, current_x, current_y, current_yaw):
    return \
"    <node pkg=\"gazebo_ros\" type=\"spawn_model\" name=\"spawn_grapevine_model_"\
 + str(current_n) + "\" args=\"-urdf -param /grapevine_"\
 + str(current_grapevine_model) + "_description -urdf -x "\
 + str(current_x) + " -y " + str(current_y) + " -z 0.0 -Y " + str(current_yaw) + " -model grapevine_" + str(current_n) + "\"/>\n"

if __name__ == '__main__':

    if(len(sys.argv) > 2):
        N_plants_in_row = int(sys.argv[1])
        N_rows = int(sys.argv[2])
        
        if(N_plants_in_row <= 0):
            raise Exception("N_plants_in_row must be 1 or greater!")

        if(N_rows <= 0):
            raise Exception("N_rows must be 1 or greater!")
    else:
        N_plants_in_row = 5
        N_rows = 4


    launch_file = open("../../viv_gazebo/launch/spawn_random_vineyard.launch", "w")

    launch_file_header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<launch>\n"
    launch_file_footer = "</launch>\n   "
    
    model_descriptions = \
"    <param name=\"grapevine_1_description\" command=\"$(find xacro)/xacro --inorder '$(find viv_description)/urdf/grapevines/grapevine.urdf.xacro'\" />\n\
    <param name=\"grapevine_2_description\" command=\"$(find xacro)/xacro --inorder '$(find viv_description)/urdf/grapevines/grapevine2.urdf.xacro'\" />\n\
    <param name=\"grapevine_3_description\" command=\"$(find xacro)/xacro --inorder '$(find viv_description)/urdf/grapevines/grapevine3.urdf.xacro'\" />\n\
"

    launch_file.write(launch_file_header)
    launch_file.write(model_descriptions)



    every_N_plant_pole = 5

    y_start = -1.0
    x_start = 2.0

    row_width = 2.0
    plant_x_distance = 1.0

    plant_yaw = math.pi/2

    launch_file.write("\n")
    current_n = 1
    for j in range(1, N_rows + 1):
        for i in range(1, N_plants_in_row + 1 ):
            current_x = x_start + (i-1)*plant_x_distance
            current_y = y_start + (j-1)*row_width

            launch_file.write(\
                spawn_model(current_n, random.randrange(1, 4), current_x, current_y, plant_yaw)\
            )

            current_n = current_n + 1
        launch_file.write("\n")

    launch_file.write(launch_file_footer)
    launch_file.close()