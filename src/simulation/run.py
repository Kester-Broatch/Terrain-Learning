#!/usr/bin/env python3

# Script to run simulation launch file multiple times

# Must source ros setup.bash file in terminal before running
# Some simulation parameters for launch file are defined here (others can be added) and defaults are in launch file
# Output locations are setup in the run_simulation.py file TODO - if time fix this

import os
import roslaunch
import rospy
import pandas as pd
import numpy as np

project_path = os.path.dirname(os.path.realpath(__file__)).replace('/src/simulation','')
launch_path = project_path+"/src/simulation/catkin_ws/src/simulate_traversability_core/launch/test.launch"

dataset_type = '_' # Optional label
number_tries = '10' # Traverses per world
gui = 'false'
headless = 'false'

world_names=['test.world']

# Run simulation for world name
for world_name in world_names:
    # Launch Arguments
    arg_world_name = 'world_name:='+world_name 
    arg_dataset_type = 'dataset_type:='+dataset_type 
    arg_number_tries = 'number_tries:='+number_tries

    arg_paused = 'paused:=false'
    arg_use_sim_time = 'use_sim_time:=true'
    arg_gui = 'gui:='+gui
    arg_headless = 'headless:='+headless
    arg_debug = 'debug:=false'
    arg_verbose = 'verbose:=true'

    # Launch simulation for given world
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args = [launch_path,arg_world_name,arg_dataset_type,arg_number_tries,arg_paused,arg_use_sim_time,arg_gui,arg_headless,arg_debug,arg_verbose]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()
    launch.spin() # Spins until run_simulation node terminates


