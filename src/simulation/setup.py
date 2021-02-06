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
    if not os.path.exists(path+'/pcloud'): os.makedirs(path+'/pcloud')

    for image_name in sorted(os.listdir(path+'/depth_gray')):  
        image_name = image_name.replace(".png","")
        rgb_raw = o3d.io.read_image(path+'/rgb/'+image_name+'.jpg') 
        depth_raw = o3d.io.read_image(path+'/depth_gray/'+image_name+'.png') 
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_raw, 
            depth_raw,
            depth_scale=0.025, # Gives max depth of around 30m  
            depth_trunc=100.0,
            convert_rgb_to_intensity=False)
        
        cam = o3d.camera.PinholeCameraIntrinsic()
        # cam.set_intrinsics(648,488,600,600,324,244) #Bumblebee stereo params
        cam.set_intrinsics(648,488,1000,1000,324,244) # Tuned for more realistic meshes

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            cam)
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        # Rotate to make ground flat (accounts for downwards pointing cam)
        pcd.rotate(pcd.get_rotation_matrix_from_xyz((-np.pi / 8, 0, 0)), 
            center=(0, 0, 0))      

        # Visualise PCD
        # X = Left (-ve) and Right (+ve) of image
        # Y = Ground height below camera, Down is -ve, Up is +ve
        # Z = Depth into image from camera, always -ve
        # origin = o3d.geometry.TriangleMesh.create_coordinate_frame(
        #     size=0.6, origin=[0, 0, 0])
        # o3d.visualization.draw_geometries([pcd] + [origin])
        # o3d.visualization.draw_geometries_with_vertex_selection([pcd])

        # Save pcd
        o3d.io.write_point_cloud(
            filename=path+'/pcloud/'+image_name+'.pcd',
            pointcloud=pcd,
            write_ascii=True)


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