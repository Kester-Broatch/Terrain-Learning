Last Modified: February 08, 2018
_____________________________________________________________________

FREIBURG MULTISPECTRAL FOREST DATASET

@author
         Abhinav Valada
         avalada@cs.uni-freiburg.de
         http://deepscene.cs.uni-freiburg.de
_____________________________________________________________________

This package contains the original dataset along with the adverse conditions subset.
The adverse conditions subset contains images captured in the presence of snow, low-lighting, glare,
and motion blur.

There are two seperate TRAINING and TEST sets that are provided. The TRAINING set contains 230 images and
the TEST set contains 136 images. In the ISER'16 paper, each image in the TRAIN set was augmented x300 times.
For details on different types of augmentations applied, please see the ISER'16 paper.

_____________________________________________________________________

If you use this dataset for research, please consider citing our ISER'16 paper:

A. Valada, G. Oliveira, T. Brox, and W. Burgard, "Deep Multispectral Semantic Scene Understanding of Forested 
Environments using Multimodal Fusion", The 2016 International Symposium on Experimental Robotics (ISER 2016), 
Tokyo, Japan, October 2016. 

@inproceedings{valada16iser,
  author = {Abhinav Valada and Gabriel Oliveira and Thomas Brox and Wolfram Burgard},
  title = {Deep Multispectral Semantic Scene Understanding of Forested Environments using Multimodal Fusion},
  booktitle = {The 2016 International Symposium on Experimental Robotics (ISER 2016)},
  year = 2016,
  month = oct,
  url = {http://ais.informatik.uni-freiburg.de/publications/papers/valada16iser.pdf},
  address = {Tokyo, Japan}
}

_____________________________________________________________________

DESCRIPTION:


This dataset contains multispectral and multimodal images. Each image in the folders correspond to a specific
view of the scene. All the images in this dataset are not of the same dimension, they need to be resized to
a fixed dimension before using. Each of the TRAIN and TEST sets contains the following data,


* rgb: 	    	   folder containing standard RGB images

* rgb_grayscale    folder containing the grayscale version of the RGB images

* depth_color      folder containing the colorized depth images as 3 channels
                   Anat Levin's colorization was applied

* depth_color_1ch  folder containing the colorized images as a 1 channel image
                   Anat Levin's colorization was applied

* depth_gray       folder containing the 1 channel depth images

* evi_color        folder containing the 3 channel colorized Enhanced Vegetation Index images
                   A modified Jet colormap was applied

* evi_gray         folder containing the 1 channel Enhanced Vegetation Index images

* ndvi_color       folder containing the 3 channel colorized Normalized Difference Vegetation Index images
                   A modified Jet colormap was applied

* ndvi_float       folder containing the Normalized Difference Vegetation Index images in floating-point

* nir              folder containing the images captured with a Wratten 25A filter
                   Near-Infrared wavelength was captured in the Blue channel and mostly red light in the
                   red channel

* nir_color        folder containing the 3 channel colorized Near-Infrared images
                   A modified Jet colormap was applied

* nir_color_jet    folder containing the 3 channel colorized Near-Infrared images
                   A standard Jet colormap was applied

* nir_gray         folder containing the 1 channel Near-Infrared images

* nrg              folder containing the 3 channel NRG (Near-InfraRed, RED, GREEN) images

* GT_color:	   folder containing the groundtruth masks for semantic segmentation 
                   Annotations are given using a color representation, where each color corresponds to a
                   specific class. This is primairly provided for visualization. For training, create a
                   corresponding ID image by assigning the colors to a speific class ID as given below



Class		R	G	B	ID


Void		- 	- 	-	0

Road            170 	170 	170	1

Grass           0 	255 	0	2

Vegetation      102 	102 	51	3

Tree            0 	60 	0	3

Sky             0 	120 	255	4

Obstacle        0 	0 	0	5

_____________________________________________________________________

CHANGELOG 08/02/2018

-Fixed incorrect label for the Obstacle class in the test groundtruth
-Fixed incorrect groundtruth for test image b275-311 and replaced it with the correct groundtruth 
-Removed duplicate b223-093 image from test images



_____________________________________________________________________
