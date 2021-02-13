#!/usr/bin/env python

from __future__ import division

import sys
import rospy
import os
import time
import math
import numpy as np
import pandas as pd
import open3d as o3d

import std_msgs.msg 
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench
from nav_msgs.msg import Odometry

from tf import transformations
from collada import Collada

## For gazebo communication
model_name = "pioneer3at"
root_relative_entity_name = '' # this is the full model and not only the base_link
fixed_twist = Twist()
fixed_twist.linear.x = 0.5 #move forward at .15 m/s
fixed_twist.linear.y = 0
fixed_twist.linear.z = 0
fixed_twist.angular.x = 0
fixed_twist.angular.y = 0
fixed_twist.angular.z = 0

sim_duration = rospy.Duration(20) # 20segs, we could also used non-ros functions with an integer

# For saving the information
output_folder = "~/dataset_cvs/"

def get_cam_pos(world_info):
    dataframe = pd.read_csv(world_info)
    cam_pose_vector = dataframe.to_numpy()
    cam_pose_vector = cam_pose_vector[0]
    cam_pose = Pose()
    cam_pose.position.x = cam_pose_vector[0]
    cam_pose.position.y = cam_pose_vector[1]
    cam_pose.position.z = cam_pose_vector[2]
    qto = transformations.quaternion_from_euler(cam_pose_vector[3], cam_pose_vector[4], cam_pose_vector[5], axes='sxyz')
    cam_pose.orientation.x = qto[0]
    cam_pose.orientation.y = qto[1]
    cam_pose.orientation.z = qto[2]
    cam_pose.orientation.w = qto[3]
    return cam_pose

def generate_pose(cam_pose, pcd_array, i_sim, number_tries):    

    # Calculate start angles at each spawn
    vectors = pcd_array - [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z]
    cam2point_angles_z = np.arctan2(vectors[:,0],vectors[:,1])
    FOV = 2*max(abs(cam2point_angles_z))
    segment_angle = (FOV)/(number_tries+1)
    start_angle_z = (-FOV/2) + ((i_sim+1)*segment_angle) # 0 degs along y axis: +ve = positive x, -ve = negative x

    # Calculate start positions 
    POI_indexs = np.rad2deg(abs(start_angle_z - cam2point_angles_z))<0.5 # indexes of points of interest (POI) near the desired angle
    POI_vectors = vectors[POI_indexs]
    POI_dists = np.linalg.norm(POI_vectors,axis=1)
    edge_offset = 3 # to avoid falling off edge
    z_offset = 0.3 # to avoid spawning inside/under mesh
    POI = POI_vectors[np.argmin(np.abs(POI_dists-POI_dists.min()-edge_offset))] + [0,0,z_offset]
    start_pose = POI + [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z]

    # Assign start positions and angles (gazebo uses 0 degs along x axis, so minus 90 degs required)
    pose = Pose()
    pose.position.x = start_pose[0]
    pose.position.y = start_pose[1]
    pose.position.z = start_pose[2]
    qto = transformations.quaternion_from_euler(0, 0, -start_angle_z+(0.5*math.pi), axes='sxyz')
    pose.orientation.x = qto[0]
    pose.orientation.y = qto[1]
    pose.orientation.z = qto[2]
    pose.orientation.w = qto[3]
    return pose

def generate_model_state(cam_pose, pcd, i_sim, number_tries):
    # generates a set of valid simulation spawn positions (advancing away from camera)

    pcd_array = np.asarray(pcd.points)

    # Divide spawn positions
    number_spawn_positions = 2
    dist_between_spawns = (pcd_array[:,1].max() - pcd_array[:,1].min())/number_spawn_positions
    sims_per_spawn_position = number_tries/number_spawn_positions
    spawn_position_label = np.arange(number_tries)//sims_per_spawn_position # Eg 0,1,2...(number_spawn_positions - 1)
    
    # Setting new cam position and cropping point cloud for this new pos
    cam_pose_spawn = Pose()
    cam_pose_spawn.position.y = cam_pose.position.y + dist_between_spawns*spawn_position_label[i_sim]
    cropped_pcd_index = pcd_array[:,1]>=(pcd_array[:,1].min()+dist_between_spawns*spawn_position_label[i_sim])
    pcd_spawn = pcd_array[cropped_pcd_index]

    # Finding number of tries at each spawn
    spawn_position_index =  np.where(spawn_position_label[i_sim]==spawn_position_label)
    number_tries_spawn = len(spawn_position_index[0])
    i_sim_spawn = np.where(spawn_position_index[0]==i_sim)[0][0]

    # Finding model states at that spawn 
    model_state = ModelState()
    model_state.pose = generate_pose(cam_pose_spawn, pcd_spawn, i_sim_spawn, number_tries_spawn)
    model_state.model_name = model_name
    model_state.twist = fixed_twist # is better to set the robot control using a publisher instead of a service that only do it once (very quickly)
    model_state.reference_frame = root_relative_entity_name
    return model_state

world_info = "/home/kester/MscProject/simulation/src/simulate_traversability/gazebo/models/meshes/b1-99445/b1-99445_info.csv"
cam_position = get_cam_pos(world_info)
world_xyz = "/home/kester/MscProject/simulation/src/simulate_traversability/gazebo/models/meshes/b1-99445/b1-99445.xyz"
pcd = o3d.io.read_point_cloud(world_xyz)
# mesh_file = "/home/kester/MscProject/simulation/src/simulate_traversability/gazebo/models/meshes/b1-99445/b1-99445.dae"
# mesh = Collada(mesh_file)
# print(mesh.geometries)
number_tries = 10

for i_sim in range(0, number_tries):
    model_state = generate_model_state(cam_position, pcd, i_sim, number_tries)
    print(model_state)
