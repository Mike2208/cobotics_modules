World Step Control (shouldn't be implemented in the NRP):
- Folder: world_step_control
- Similar to IBA, used for timing synchronization between ROS nodes
- Available for C++ and Python
- Differences: 
	- Enables pause time between executions and variable execution times
	- Runs in Gazebo instead of the CLE
	- Data is exchanged between modules via ROS topics/services. We don't use a data exchange mechanism similar to the one in the IBA
- If you find a class that inherits from world_step_control.StepModule, that can be ported into the IBA
	- Porting requirements (timing and data exchange):
		- ExecuteStep is the function that is called at the requested timesteps
		- Timing: Our module let's you dynamically change the execution step timing and also add pauses between them. Fortunately, we don't use that functionality. So to set the execution timestep in the IBA, look at the return value of ExecuteStep. It should be a class called world_step_control.ModuleExecutionResult(). Use ExecutionTime value (world_step_control.ModuleExecutionResult.ExecutionTime) as the constant timestep in the IBA
		- Data is exchanged between modules via ROS topics/services, we don't use a data exchange mechanism similar to the one in the IBA. So you should be able to simply use that as well, without any changes


Intuitive movement:
- kuka_iiwa_hbp
- Generates trajectories to move a kuka iiwa robot arm "intuitively", similar to how humans move their hands
- Relevant file: kuka_iiwa_control_hbp/scripts/trajectory_generation.py
- NOTE THE COMMENTS REGARDING EXECUTION IN THE NRP
- Input: geometry_msgs.msg.Pose from ROS topic "/iiwa/trajectory_generation/goal"
- OUtput: kuka_iiwa_control.msg.DesiredTrajectory to ROS topic "/iiwa/trajectory_generation/desired_ee_trajectory"
- Timing: Managed internally
- Testing launch file: kuka_iiwa_gazebo/launch/kuka_iiwa.launch


Saliency Module:
- saliency_module
- Python module that takes a camera frame as input and detects its saliency
- Relevant file: scripts/run_saliency.py
- Uses world_step_control for timing synchronization
- Input:  sensor_msgs.msg.Image from ROS topic "/camera/camera/image"
- Output: sensor_msgs.msg.Image to ROS topic "saliency/image"
- Timing: Executes at a rate of 60Hz
- Python dependencies: tensorflow (tested on version 1.13), opencv_python, rospy, cv_bridge, (world_step_control)
- Requires CUDA for tensorflow
- Launch argument suggestions:
	- "camera_input": Name of ROS topic that sends camera image
	- "output": Name of ROS topic to send generated saliency image to
	- "gpu_factor": Percentage of GPU resources to allocate (from 0.0 to 1.0)
	
Prednet Module:
- prednet_segmentation
- Python module that takes a camera frame and the saliency data from saliency_module as inputs, then detects pre-trained objects in the camera image. It generates an image where each of the detected objects are given a predefined, unique color
- Relevant file: scripts/run_prednet.py
- Uses world_step_control for timing synchronization
- Input:  
	- sensor_msgs.msg.Image from ROS topic "/camera/camera/image"
	- sensor_msgs.msg.Image from ROS topic "/saliency/image"
- Output:
	- sensor_msgs.msg.Image to ROS topic "segmentation/image"
- Timing: Executes at a rate of 60Hz
- Python dependencies: pytorch (tested on version 1.8.1), opencv_python, rospy, matplotlib, (world_step_control)
- Requires CUDA for pytorch
- Launch argument suggestions:
	- "camera_input":   Name of ROS topic that sends camera image
	- "saliency_input": Name of ROS topic that sends saliency image
	- "output": Name of ROS topic to send generated segmentation image to
	- "gpu_factor": Percentage of GPU resources to allocate (from 0.0 to 1.0)

NOTE REGARDING PYTORCH AND TENSORFLOW:
We had to use venvs to get the correct versions of pytorch and tensorflow running. Starting the ROS nodes in their respective venvs with roslaunch required an additional execution script, see scripts/saliency.sh and scripts/prednet.sh for examples. In the roslaunch file, these commands would be executed via
	<node name="saliency_module"      pkg="saliency_module"      type="saliency.sh" output="screen"/>
	<node name="prednet_segmentation" pkg="prednet_segmentation" type="prednet.sh"  output="screen"/>




Neuromusculoskeletal model:
- edlut
- This is a tricky model. It contains three items, a musculoskeletal model (basically, a model of a human upper body's sekleton, with only the right shoulder and elbow joints moving), a cerebellar model, and a spinal cord model. Internally, the musculoskeletal model is simulated in OpenSim, and the cerebellar and spinal cord model control control the muscles that move the forearm and upper arm.
- Porting problems: We use workd_step_control for synchronization again, but here some of the code is in C++, not python. As far as I'm aware, there's no IBA C++ module available at the moment. 
- See README in edlut for details about setup and installation
- NOTE: The original files had many references to fixed absolute locations inside the filesystem. This made running the program difficult, because most files weren't in their expected locations. I should have fixed all of them, but you could run "grep -nr '/home/'" to check if I missed any.
- INSTALLATION ADDENDUM: Take a look at build.sh to see how I installed EDLUT. The original version had it installed at some random location in the filesystem, which made debugging difficult. By running the 
autoconf
./configure ....
make ....
make install ....
commands like this, EDLUT is installed in the devel directory of this ROS workspace, and can be easily found by ROS nodes
