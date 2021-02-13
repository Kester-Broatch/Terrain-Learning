#!/usr/bin/env python

# @author omar
# Edited by kester broatch

#
# It can be run alone:
# rosrun simulate_traversability_core run_simulation.py world_name dataset_type number_tries
#
# Or using the roslaunch file:
#    roslaunch simulate_traversability_core launch_simulations.launch world_name:=custom1 dataset_type:=training number_tries:=50
#
# We can deactivate the gui to speed up the simulation data gathering and 
# make the simulator run headless as such: 
#    roslaunch simulate_traversability_core launch_simulations.launch headless:=true gui:=false world_name:=custom1 dataset_type:=training number_tries:=20

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

## Constant definitions for training/evaluation map simulations
# in m, heightmaps (in simulation) are assumed "squared" (sic) so min_hm_x is equal to -max_hm_x
max_hm_x = 5.0
max_hm_y = 5.0
# max map height, 0 is the min
max_hm_z = 1.0
# in pixels, heightmaps (images) are asusmed squared, gazebo requires sizes 2^n+1 x 2^n+1
im_hm_x = 513
im_hm_y = 513

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
output_folder = "~/MscProject/training/data/sim_csvs/"
image_output_folder = "/home/kester/MscProject/training/data/sim_images/" 

def generate_random_pose():
    # x,y (z will be fixed as the max_hm_z so that the robot will drop down), gamma as orientation
    rn = np.random.random_sample((3,))
    random_pose = Pose()
    random_pose.position.x = 2 * max_hm_x *rn[0] - max_hm_x
    random_pose.position.y = 2 * max_hm_y *rn[1] - max_hm_y
    random_pose.position.z = max_hm_z * 0.5 # spawn not at the greatest height to avoid getting upside down when it falls on a tricky obstacle
    qto = transformations.quaternion_from_euler(0, 0, 2*math.pi * rn[1], axes='sxyz')
    random_pose.orientation.x = qto[0]
    random_pose.orientation.y = qto[1]
    random_pose.orientation.z = qto[2]
    random_pose.orientation.w = qto[3]
    return random_pose

def generate_random_model_state():
    random_pose = generate_random_pose()
    model_state = ModelState()
    model_state.model_name = model_name
    model_state.pose = random_pose
    model_state.twist = fixed_twist # is better to set the robot control using a publisher instead of a service that only do it once (very quickly)
    model_state.reference_frame = root_relative_entity_name
    return model_state

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

def generate_pose(cam_pose, pcd, i_sim, number_tries):    
    
    o3d.geometry.PointCloud.points

    # Calculate start angles
    vectors = np.asarray(pcd.points) - [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z]
    cam2point_angles_z = np.arctan2(vectors[:,0],vectors[:,1])
    FOV = 2*max(abs(cam2point_angles_z))
    segment_angle = (FOV)/(number_tries+1)
    start_angle_z = (-FOV/2) + ((i_sim+1)*segment_angle) # 0 degs along y axis: +ve = positive x, -ve = negative x

    # Calculate start positions 
    POI_indexs = np.rad2deg(abs(start_angle_z - cam2point_angles_z))<0.5 # indexes of points of interest (POI) near the desired angle
    POI_vectors = vectors[POI_indexs]
    POI_dists = np.linalg.norm(POI_vectors,axis=1)
    POI = POI_vectors[np.argmin(POI_dists)]
    offset = [2*np.sin(start_angle_z), 2*np.cos(start_angle_z), 0.5]
    start_pose = POI + offset + [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z]

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
    # generates a set of valid simulation starting positions (advancing away from camera)

    model_state = ModelState()
    model_state.model_name = model_name
    model_state.pose = generate_pose(cam_pose, pcd, i_sim, number_tries)
    model_state.twist = fixed_twist # is better to set the robot control using a publisher instead of a service that only do it once (very quickly)
    model_state.reference_frame = root_relative_entity_name
    return model_state

def get_model_state():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name, root_relative_entity_name)
        return resp1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)
        return False

def set_model_state(model_state):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp1 = gms(model_state)
        return resp1
    except rospy.ServiceException as e:
        #print "Service call failed: %s"%e
        rospy.logerr("Service call failed: %s"%e)

def is_pose_in_map(pose):
    # little_offset = 0.1 # to avoid waiting until the robot is really on the edge
    # if pose.position.x > max_hm_x-little_offset or pose.position.x < -max_hm_x+little_offset:
    #     return False
    # if pose.position.y > max_hm_y-little_offset or pose.position.y < -max_hm_y+little_offset:
    #     return False
    if pose.position.z < -1.5: # beacuse of urdf confituration, the z 0 is ~ -0.02
        return False
    return True

def usage():
    return "%s [world_name] [number_of_tries]"%sys.argv[0]

## class to manage the publishers and subscirbers to send and recover data from simulation using rostopics instead of services
class PubsSubsManager:
    def __init__(self):
        self.send_cmd_vel_ = False
        self.process_odom_ = False
        # self.odom_topic_ = "/sim_p3at/odom"
        # self.cmd_vel_topic_ = "/sim_p3at/cmd_vel"
        # rospy.Subscriber(self.odom_topic_, Odometry, self.callback_odom)
        self.odom_topic_ = "/gazebo/model_states"
        rospy.Subscriber(self.odom_topic_, ModelStates, self.callback_odom)
        self.cmd_vel_topic_ = "/sim_p3at/cmd_vel"
        self.pub_cmd_vel_ = rospy.Publisher(self.cmd_vel_topic_, Twist, queue_size=10)
        #while not rospy.is_shutdown():
        #    if self.send_cmd_vel_:
        #        self.publish_cmd_vel()
        
        # for storing on csv
        self.world_name_ = "generic_world"
        self.dataset_type_ = "training"
        self.i_sim_ = 0
        self.starting_time_ = 0
        self.initial_pose_ = 0
        self.data_buffer_ = 0

    def set_flag_cmd_vel(self, val):
        if val == True:
            self.send_cmd_vel_ = val
        else:
            self.send_cmd_vel_ = False

    def set_flag_odom(self, val):
        if val == True:
            self.process_odom_ = val
        else:
            self.process_odom_ = False

    def get_flag_cmd_vel(self):
        return self.send_cmd_vel_

    def get_flag_odom(self):
        return self.process_odom_
    
    def publish_cmd_vel(self):
        #h = std_msgs.msg.Header()
        #h.stamp = rospy.Time.now()
        msg = fixed_twist
        self.pub_cmd_vel_.publish(msg)
        
    # here we read the odom and store it on a cvs file
    def callback_odom(self, msg):
        model_index = msg.name.index(model_name) 
        current_time = rospy.Time.now()
        if is_pose_in_map(msg.pose[model_index]) == False:
            self.process_odom_= False
        if self.process_odom_:
            gms = get_model_state()
            if is_pose_in_map(msg.pose[model_index]) and gms.success:
                # store the information from odom
                orientation_euler = transformations.euler_from_quaternion([
                                        gms.pose.orientation.x, 
                                        gms.pose.orientation.y, 
                                        gms.pose.orientation.z, gms.pose.orientation.w
                                        ], axes='sxyz')
                self.data_buffer_.loc[len(self.data_buffer_)] = [
                                        (current_time - self.starting_time_).to_sec(),
                                        self.initial_pose_.position.x,
                                        self.initial_pose_.position.y,
                                        self.initial_pose_.position.z,
                                        gms.pose.position.x,
                                        gms.pose.position.y,
                                        gms.pose.position.z,
                                        orientation_euler[0],
                                        orientation_euler[1],
                                        orientation_euler[2],
                                        0,
                                        gms.twist.linear.x,
                                        gms.twist.linear.y,
                                        gms.twist.linear.z,
                                        gms.twist.angular.x,
                                        gms.twist.angular.y,
                                        gms.twist.angular.z
                                        ]
            else:
                self.process_odom_= False
    
    # initialize the buffer that will store all the information
    def start_new_simulation_try(self, world_name, dataset_type, i_sim, initial_pose):
        self.world_name_ = world_name
        self.dataset_type_ = dataset_type
        self.i_sim_ = i_sim
        self.initial_pose_ = initial_pose
        # I_RIP imageframe robot initial position, S_ simulation frame, RCP robot current position, TLV twist linear vel, S_RCO_A simulation robot current orientation alpha/beta/gamma (orientation is given in euler angles
        # in the future we could add information regarding wheels speed, torque, etc
        columns = [ "TIMESTAMP",
                    "S_RIP_X",
                    "S_RIP_Y",
                    "S_RIP_Z",
                    "S_RCP_X",
                    "S_RCP_Y",
                    "S_RCP_Z",
                    "S_RCO_A",
                    "S_RCO_B",
                    "S_RCO_G",
                    "S_RC_STEER",
                    "S_RC_TLV_X",
                    "S_RC_TLV_Y",
                    "S_RC_TLV_Z",
                    "S_RC_TAV_X",
                    "S_RC_TAV_Y",
                    "S_RC_TAV_Z"
                   ]
        self.data_buffer_ = pd.DataFrame(columns = columns)
        #self.data_buffer_ = self.data_buffer_.set_index(["TIMESTAMP"])
        self.starting_time_ = rospy.Time.now()
        self.set_flag_cmd_vel(True)
        self.publish_cmd_vel()
        self.set_flag_odom(True)
     
    # save the buffer to a csv and stop the try simulation
    def stop_simulation_try(self):
        #self.data_buffer_ = self.data_buffer_.set_index("TIMESTAMP")
        rospy.loginfo("saving simulation data to %s",output_folder+self.world_name_+"_"+self.dataset_type_+"_"+str(self.i_sim_)+".csv")
        self.data_buffer_.to_csv(output_folder+self.world_name_+"_"+self.dataset_type_+"_"+str(self.i_sim_)+".csv")
        return self.world_name_+"_"+self.dataset_type_+"_"+str(self.i_sim_)+".csv"
    
if __name__ == "__main__":
    if len(sys.argv) >= 7:
        world_name = sys.argv[1]
        dataset_type = sys.argv[2]
        number_tries = int(sys.argv[3])
        world_info = sys.argv[4]
        world_xyz = sys.argv[5]
        world_png = sys.argv[6]
    else:
        rospy.logerr(usage())
        sys.exit(1)
    rospy.init_node('run_simulation_manager')
    # we wait until gazebo is up and our robot-s model is running
    while get_model_state().success==False:
        rospy.loginfo("Waiting for %s model to be up in gazebo", model_name)
        rospy.sleep(1.)
    pubssubs_manager = PubsSubsManager()
    meta_csv_buffer = pd.DataFrame(columns = ["CSV","HEIGHTMAP"])
    cam_pose = get_cam_pos(world_info)
    pcd = o3d.io.read_point_cloud(world_xyz)
    for i_sim in range(0, number_tries):
        rospy.loginfo("=== Simulation %s/%s for map %s", str(i_sim), str(number_tries), world_name)
        model_state = generate_model_state(cam_pose, pcd, i_sim, number_tries)
        rospy.loginfo("-- spawning robot at %s", model_state.pose)
        res = set_model_state(model_state)
        rospy.loginfo("-- %s", res)
        rospy.sleep(0.5)
        pubssubs_manager.start_new_simulation_try(world_name, dataset_type, i_sim, model_state.pose)
        start_t = rospy.Time.now()
        rospy.loginfo("-- listening for %s s", str(sim_duration.to_sec()) )
        while pubssubs_manager.get_flag_odom():
            current_t = rospy.Time.now()
            if (current_t - start_t) >= sim_duration:
                pubssubs_manager.set_flag_odom(False)
        cvs_file_name = pubssubs_manager.stop_simulation_try()
        meta_csv_buffer.loc[len(meta_csv_buffer)] = [ cvs_file_name, world_name+".png"]
    
    # Save meta file csv
    meta_csv_buffer.to_csv(output_folder+"meta/meta_"+world_name+"_"+dataset_type+".csv",index_label='ID')
    # Save mesh image
    im = skimage.io.imread(world_png)
    skimage.io.imsave(image_output_folder+world_name+".png",im)
    # Save image info csv file 
    im_info = pd.read_csv(world_info)
    im_info.to_csv(image_output_folder+world_name+"_info.csv")

        
