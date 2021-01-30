#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Sets up the simulations by generating collada (DAE) files from the dataset depth images. These collada files are used as terrain surfaces in the gazebo simultion.

import os
import re
import numpy as np
from PIL import Image
import open3d as o3d
import matplotlib.pyplot as plt

project_path = os.path.dirname(os.path.realpath(__file__)).replace('/src/simulation','')
data_paths = [project_path+"/data/test", project_path+"/data/train"] # Relative to this file

def RGBD2pcloud(path):
    for image_name in sorted(os.listdir(path+'/depth_gray')):  
        image_name = image_name.replace(".png","")
        rgb_raw = o3d.io.read_image(path+'/rgb/'+image_name+'.jpg') 
        depth_raw = o3d.io.read_image(path+'/depth_gray/'+image_name+'.png') 
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_raw, depth_raw)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        # o3d.visualization.draw_geometries([pcd], zoom=0.5)

def resize_images(path):
    for image_name in sorted(os.listdir(path+'/rgb')):
        image_name = image_name.replace(".jpg","")

        # We get rgb size
        rgb = Image.open(path+'/rgb/'+image_name+'.jpg')
        w = rgb.width
        h = rgb.height

        # We resize all images to rgb size
        print(image_name)
        dirnames = [dI for dI in os.listdir(path) if os.path.isdir(os.path.join(path,dI))]
        for dirname in dirnames:
            if dirname!='rgb':
                if dirname in ['evi_gray','ndvi_float','nir_gray','rgb_grayscale']:
                    filename = path+'/'+dirname+'/'+image_name+'.tif'
                elif dirname in ['ndvi_color','nrg']:
                    filename = path+'/'+dirname+'/'+image_name+'.jpg'
                else:
                    filename = path+'/'+dirname+'/'+image_name+'.png'
                img = Image.open(filename)
                img = img.resize((w,h))
                img.save(filename)

def clean_names(path):
    for (dirpath, dirnames, filenames) in os.walk(path):
        for filename in filenames:
            if '_' in filename and filename!='.DS_Store': # Removes additional naming assigned to some of the data
                underscore = filename.find("_")
                dot = filename.find(".")
                new_filename = filename[0:underscore]+filename[dot:len(filename)]
                os.rename(dirpath+"/"+filename, dirpath+"/"+new_filename)
                # print(filename + "  ---->  " + new_filename)
          
def main():
    for data_path in data_paths:

        print("Cleaning file names...")
        clean_names(data_path)

        print("Resizing Images...")
        # resize_images(data_path)

        print("Calculating point clouds... ")    
        RGBD2pcloud(data_path)


if __name__ == "__main__":
    main()