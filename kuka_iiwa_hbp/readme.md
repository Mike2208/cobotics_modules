# kuka\_iiwa\_control\_hbp

The "core" of the code is contained inside the kuka\_iiwa\_control\_hbp package.

This folder contains the scripts necessary to run the following nodes.

## Controller

This node runs the inverse kinematics and torque control of the robot.

The robot's URDF is imported directly from the /robot_description parameter loaded on the ROS parameters server.

The parameters of the controller can be changed via dynamic reconfigure.

The controller can generate both torque or position command. To change the command being generated uncomment the lines indicated by the comments in the code.
**Note 1**: the controller interface for the robot's joints, loaded by the launch file kuka\_iiwa\_control\_hbp/launch/kuka\_iiwa\_control\_hbp.launch should be changed accordingly. The controllers are defined in kuka\_iiwa\_control\_hbp/cfg/controllers.yaml .
**Note 2**: the PID's gains of the EffortController/JointPositionGroupController are just indicatively tuned, currently.

## Trajectory Generation

This node generates the trajectory desired for the robot's end-effector.

The position trajectory is generated using a Dynamic Movement Primitive, implemented in DMP\_R2H.py
The orientation trajectory is generated using a first-order dynamical system in quaternion space. The method is implemented in OrientationDynamics.py.

The parameters of both systems can be changed via dynamic reconfigure.

A new goal can be imposed to the systems by publishing the desired pose on the topic /iiwa/trajectory\_generation/goal
