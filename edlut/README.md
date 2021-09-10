# cerebellum-spinalCord-controller
Cerebellar-SNN integrated with a spinal cord model to control a musculoskeletal arm model in OpenSim. The repository includes EDLUT simulator source code for the cerebellar SNN, the ROS package integrating the cerebellum, spinal cord, and OpenSim model, and configuration files needed. This work is a collaboration between the Applied Computational Neuroscience group (Prof. Eduardo Ros, UGR) and the Biorobotics Laboratory (Prof. Auke Ijspeert, EPFL). 

##  Requirements
* A computer with at least 8GB of RAM, a multicore CPU, an NVIDIA GPU with CUDA support. 
* Ubuntu 16.04 and ROS Kinetic (the ROS package was developed for ROS Kinetic, other ROS distributions have not been tested).
* OpenSim installation: https://github.com/opensim-org/opensim-core#on-ubuntu-using-unix-makefiles (install python scripting for python 2.7)
* Install the following for spinal cord network: 
  * farms_pylog: https://gitlab.com/farmsim/farms_pylog
    * $ cd farms_pylog 
    * $ pip install -e .
  * $ pip install cython
  * farms_container: https://gitlab.com/farmsim/farms_container
    * $ cd farms_container 
    * $ pip install -e .
  * farms_network: https://gitlab.com/farmsim/farms_network
    * $ cd farms_network 
    * git checkout 29-python2-7
    * pip install -e .

## Installation
* Install EDLUT simulator (this step requires an NVIDIA GPU with CUDA support and CUDA installation):
	* Open a terminal and go to the folder EDLUT_source_code:
	* $ chmod u+x configure
	* $ ./configure
	* $ make
	* $ sudo bash
	* $ make install

* Compile the ROS package (this step requires a ROS Kinetic installation: http://wiki.ros.org/kinetic/Installation/Ubuntu)
	* Open a terminal and go to the ROS_OpenSim folder:
	* $ catkin_make

* Copy the files in /cerebellum-spinalCord-controller/config_files/neuron_models to ~/.ros/data

## Execution
* Cerebellum-SpinalCord control of elbow flexion-extension movement: 
  * $ cd /cerebellum-spinalCord-controller/ROS_OpenSim
  * $ source devel/setup.bash
  * $ roslaunch edlut_ros_osim cerebellar_spinalCordNetwork_control_elbow_flexion.launch 
  * *Note*: edit the .launch file so the input files point to a location in your computer. E.g. at line 47: param name="positions_file_name" value="/home/baxter/cerebellum-spinalCord-controller/ROS_OpenSim/src/edlut_ros_osim/elbow_flexion_trajectory/elbow_flexion_trajectory_joint_position_rad_oneJoint.txt" type="str". Change the path assigned to *value* according to your workspace. You can search "/home/baxter/" within the .launch file to locate all input files and modify them. 
    


