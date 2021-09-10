#!/bin/bash

unset SHELL_NAME

#export ROS_MASTER_URI=http://localhost:11411
#export DISPLAY=:1

#source $COB_LAUNCH_DIR/source.sh
source $COB_VENV_DIR/py_37_tensorflow_1_13/bin/activate

exec rosrun saliency_module run_saliency.py "$@"
