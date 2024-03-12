# Open Containability Imagination

# Author: Wan Su
# Institution: National University of Singapore
# Date: Mar 1, 2024

# The code is modified from Hongtao Wu's container-imagine

from __future__ import print_function

import os
import time
import argparse
from datetime import date

import numpy as np

from containability import Containability
from find_vector import Find
from normal_detection import Detection
from alignment import Alignment
from match_imagination import Match

# Visualization
visualization = True
if visualization:
    print("Visualization is activated...")

# Root directory of the code repo
root_dir = "/home/sylvie/PycharmProjects/match_imagination" # please modify this if you want to run it
content_urdf = os.path.join(root_dir, "object/m&m.urdf")

################# Containability Imagination on Cup #################
data_root_dir = "/home/sylvie/PycharmProjects/match_imagination/data" # please modify this if you want to run it
data_name = "1"
data_dir = os.path.join(data_root_dir, data_name)
mesh_name = "cup"
obj_vhacd_file = mesh_name + "_vhacd.obj"
obj_urdf = os.path.join(data_dir, mesh_name + ".urdf")

print("Start containability imagination on: {}".format(mesh_name))
obj_vhacd_path = os.path.join(data_root_dir, data_name, obj_vhacd_file)

C_cup = Containability(obj_urdf,
                        obj_vhacd_path,
                        content_urdf=content_urdf,
                        obj_zero_pos=[0, 0, 1],
                        obj_zero_orn=[0, 0, 0],
                        check_process=visualization)

containability_affordance_cup, sphere_in_percentage_cup = C_cup.get_containability()
sphere_in_list_cup = np.array(C_cup.sphere_in_drop_pos)

x_num_cup, y_num_cup, contact_pos_cup, contact_id_cup = C_cup.find_contact()
drop_in_pos_cup = C_cup.sphere_in_drop_pos
drop_in_id_cup = C_cup.sphere_in_id

C_cup.disconnect_p()

F_cup = Find(x_num_cup, y_num_cup, contact_pos_cup, contact_id_cup, drop_in_pos_cup, drop_in_id_cup)
pos_cup = F_cup.pick_center()
pos_cup[2] = pos_cup[2] - 0.01
sphere_drop_pos = C_cup.sphere_drop_pos
print(pos_cup)
F_cup.draw_spheres()
position_footprint_cup = F_cup.position_footprint

################# Container Mouth Detection #################
Detection_cup = Detection(contact_id_cup,
                          drop_in_id_cup,
                          contact_pos_cup,
                          x_num_cup,
                          y_num_cup,
                          pos_cup)
Detection_cup.detection_start()

################# Containability Imagination on Lid #################
data_root_dir = "/home/sylvie/PycharmProjects/container_imagine-master/data" # please modify this if you want to run it
data_name = "1"
data_dir = os.path.join(data_root_dir, data_name)
mesh_name = "lid"
obj_vhacd_file = mesh_name + "_vhacd.obj"
obj_urdf = os.path.join(data_dir, mesh_name + ".urdf")

print("Start containability imagination on: {}".format(mesh_name))
obj_vhacd_path = os.path.join(data_root_dir, data_name, obj_vhacd_file)

C_lid = Containability(obj_urdf,
                           obj_vhacd_path,
                           content_urdf=content_urdf,
                           obj_zero_pos=[0, 0, 1],
                           obj_zero_orn=[0, 0, 0],
                           check_process=visualization)

containability_affordance_lid, sphere_in_percentage_lid = C_lid.get_containability()
sphere_in_list_lid = np.array(C_lid.sphere_in_drop_pos)

x_num_lid, y_num_lid, contact_pos_lid, contact_id_lid = C_lid.find_contact()
drop_in_pos_lid = C_lid.sphere_in_drop_pos
drop_in_id_lid = C_lid.sphere_in_id

C_lid.disconnect_p()

F_lid = Find(x_num_lid, y_num_lid, contact_pos_lid, contact_id_lid, drop_in_pos_lid, drop_in_id_lid)
pos_lid = F_lid.pick_center()
pos_lid[2] = pos_lid[2] - 0.01

print(pos_lid)
F_lid.draw_spheres()
positon_footprint_lid = F_lid.position_footprint

################# Footprint Alignment #################
A = Alignment(position_footprint_cup, positon_footprint_lid)
A.alignment_start()
best_angle = A.best_angle

################ Match Imagiantion #################
M = Match(pos_cup, pos_lid, best_angle)
M.start_match()