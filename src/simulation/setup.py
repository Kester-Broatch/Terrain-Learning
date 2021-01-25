#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Sets up the simulations by generating collada (DAE) files from the dataset depth images. These collada files are used as terrain surfaces in the gazebo simultion.

import os
import re
import numpy as np
from open3d import camera, geometry, io
import matplotlib.pyplot as plt

project_path = os.path.dirname(os.path.realpath(__file__)).replace('/src/simulation','')
data_paths = [project_path+"/data/test", project_path+"/data/train"] # Relative to this file

def clean_names(path):
    print("Cleaning file names in ... "+path)
    for (dirpath, dirnames, filenames) in os.walk(path):
        for filename in filenames:
            if '_' in filename and filename!='.DS_Store': # Removes additional naming assigned to some of the data
                underscore = filename.find("_")
                dot = filename.find(".")
                new_filename = filename[0:underscore]+filename[dot:len(filename)]
                os.rename(dirpath+"/"+filename, dirpath+"/"+new_filename)
                # print(filename + "  ---->  " + new_filename)

def depth2pcloud(path):
    print("Calculating point clouds from ... "+path)    
    phc = camera.PinholeCameraIntrinsic()
    for (dirpath, dirnames, filenames) in os.walk(path):  
        for filename in filenames:
            depth = io.read_image(dirpath+"/"+filename) 
            np_depth = np.array(depth)
            phc.set_intrinsics(np_depth.shape[1],np_depth.shape[0],3.8,3.8,np_depth.shape[1]/2,np_depth.shape[0]/2)
            pcloud = geometry.PointCloud()
            pcloud.create_from_depth_image(np_depth, phc)

          
def main():
    for data_path in data_paths:
        clean_names(data_path)
        depth2pcloud(data_path+"/depth_gray")


if __name__ == "__main__":
    main()