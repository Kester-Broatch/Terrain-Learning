# Terrain Learning
This repo contains the code I used in my MSc project to learn the vehicle dynamics of a simulated rover over some terrain which was generated from a real data set. More details on the project can be obtained from https://www.kesterbroatch.com/projects/terrain-learning-vehicle

This repo is organised as follows:

```
├── data----------------Images and 3d files required to generate terrain simulations and train the model
│   ├── test
│   └── train
|
├── models--------------Weights of trained tensorflow models
|
├── src
│   ├── simulation------Tools to generate and run terrain simulations with the gazebo physics engine
│   ├── processing------Tools to generate training data from simulation results 
│   └── training--------Scripts to train the model to predict terrain traversability  
|
└── tests---------------Model performance and code unit tests
```

### Dependancies
* ROS melodic (ubuntu 18.04) 
* Python3 with the open3d, tensorflow, numpy, PIL and matplotlib packages.

### Download Dataset 
The first step is to download the deep scene terrain dataset from the university of freigburg using: 
```
./data/download-data.sh
```
This dataset is 1.7GB and could take a while to download.

### Generate Simulation Data
The depth images from the dataset must now be converted into mesh files which can be used by the gazebo simulator:
```
python3 src/simulation/setup.py
```

### Build simulation ROS package
1. Setup your ROS installation (if not already done), eg for melodic:

```
source /opt/ros/melodic/setup.bash
```

1. Build the simulation packages inside the catkin workspace:

```
catkin_make --directory src/simulation/catkin_ws/
```

1. Source the catkin workspace to ensure that the packages can be located by ROS (this step should be done in every new terminal in which you want to use the packages, or added to bashrc if you want it to be permanent):

```
source src/simulation/catkin_ws/devel/setup.bash
```

The simulation packages should now be compiled and added to your ros package path. You can confirm that they are there using:

```
rospack list | grep simulate_traversability
```
