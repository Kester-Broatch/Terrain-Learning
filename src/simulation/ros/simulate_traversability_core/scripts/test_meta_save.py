#!/usr/bin/env python


import sys
import rospy
import os
import time
import math
import numpy as np
import pandas as pd
import open3d as o3d
import skimage
import skimage.io

import std_msgs.msg 
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench
from nav_msgs.msg import Odometry

from tf import transformations

# For saving the information
output_folder = "~/MscProject/training/data/sim_csvs/"
image_output_folder = "/home/kester/MscProject/training/data/sim_images/" 

name_list = "/home/kester/MscProject/simulation/src/simulate_traversability/gazebo/models/meshes/Mesh List Test.csv" # list of the meshes to test (allows batches to be run)

dataset_type = 'testing'
number_tries = 10 # Traverses per world

# Obtain mesh names 
mesh_list=pd.read_csv(name_list)

# Run simulation for world name
for world_name in mesh_list.name:
    print "World = "+str(world_name)+"...."+str(np.where(mesh_list.name==world_name)[0][0])+"/"+str(len(mesh_list.name))
    world_info = "/home/kester/MscProject/simulation/src/simulate_traversability/gazebo/models/meshes/"+world_name+"/"+world_name+"_info.csv"
    world_png = "/home/kester/MscProject/simulation/src/simulate_traversability/gazebo/models/meshes/"+world_name+"/"+world_name+".png"

    meta_csv_buffer = pd.DataFrame(columns = ["CSV","HEIGHTMAP"])
    for i_sim in range(0, number_tries):
        cvs_file_name = world_name+"_"+dataset_type+"_"+str(i_sim)+".csv"
        meta_csv_buffer.loc[len(meta_csv_buffer)] = [ cvs_file_name, world_name+".png"]

    # Save meta file csv
    meta_csv_buffer.to_csv(output_folder+"meta/meta_"+world_name+"_"+dataset_type+".csv",index_label='ID')
    # Save mesh image
    im = skimage.io.imread(world_png)
    skimage.io.imsave(image_output_folder+world_name+".png",im)
    # Save image info csv file 
    im_info = pd.read_csv(world_info)
    im_info.to_csv(image_output_folder+world_name+"_info.csv")


            
