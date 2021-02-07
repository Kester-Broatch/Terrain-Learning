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
This project requires either linux or macos, with docker and docker-compose. In additionto this, it requires python3 with the open3d, tensorflow, numpy, PIL and matplotlib packages.

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

### Build and Run Simulation
The simulation is built and run using docker containers, which are mounted the `data` directory. To start the simulation running in headless mode (no GUI) use:
```
./src/simulation/run.sh 
```
To mount the simulator onto you graphics socket and view the simulation through a gui use:
```
./src/simulation/run.sh -g
```
The simulator will save it's output in the data directory.
