#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Projects/opensim/install/lib
export SIMBODY_HOME=~/Projects/opensim/install
export OPENSIM_HOME=~/Projects/opensim/install/
export PYTHONPATH=$PYTHONPATH:~/Projects/opensim/install/lib/python3.7/site-packages

# Source osim venv
source ~/Projects/Cobotics_Experiment/venvs/opensim_edlut/bin/activate

exec roslaunch edlut_ros_osim cerebellar_spinalCordNetwork_control_elbow_flexion.launch "$@"
