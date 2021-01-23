# Terrain Learning
This repo contains the code I used in my MSc project to learn the vehicle dynamics of a simulated rover over some terrain which was generated from a real data set. The original thesis can be downloaded from - https://www.kesterbroatch.com/assets/terrain-learning-vehicle/Learning_Vehicle_Dynamics_Models_by_Self_Supervised_Learning.pdf

This repo is organised as follows:

```
├── data
│   ├── download-data.sh
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
This dataset is 1.7GB large and will take a few minutes to download.