# Terrain Learning
This repo contains the code I used in my MSc project to learn the vehicle dynamics of a simulated rover over some terrain which was generated from a real data set. More details on the project can be obtained from https://www.kesterbroatch.com/projects/terrain-learning-vehicle

This repo is organised as follows:

```
├── data
│   ├── test
│   └── train
├── models
├── src
│   ├── processing
│   ├── simulation
│   └── training
└── tests
```

### Download Dataset 
The first step is to download the deep scene terrain dataset from the university of freigburg using: 
```
./data/download-data.sh
```
This dataset is 1.7GB and could take a while to download.

### Generate Simulation Data
The depth images from the dataset must now be converted into a 3D DAE mesh file which can be used by the simulation:
```
python3 src/simulation/setup.py
```

