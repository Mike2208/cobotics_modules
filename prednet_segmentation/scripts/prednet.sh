#!/bin/bash

#export ROS_MASTER_URI=http://localhost:11411
#export DISPLAY=:1

#unset SHELL_NAME

#source $COB_LAUNCH_DIR/source.sh
source $COB_VENV_DIR/pytorch_1_8_1/bin/activate

exec rosrun prednet_segmentation run_prednet.py "$@"
