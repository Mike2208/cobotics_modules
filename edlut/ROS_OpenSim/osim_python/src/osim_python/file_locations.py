import rospkg

rospack = rospkg.RosPack()

edlut_ros_osim_path = rospack.get_path("edlut_ros_osim")

elbow_flexion_one_joint_pos = edlut_ros_osim_path + "/elbow_flexion_trajectory/elbow_flexion_trajectory_joint_position_rad_oneJoint.txt"
elbow_flexion_one_joint_vel = edlut_ros_osim_path + "/elbow_flexion_trajectory/elbow_flexion_trajectory_joint_velocity_rad_oneJoint.txt"
traj_elbow_flexion_one_joint_pos = edlut_ros_osim_path + "../TrajectoryGeneration/elbow_flexion_trajectory/elbow_flexion_trajectory_joint_position_rad.txt"
traj_elbow_flexion_one_joint_vel = edlut_ros_osim_path + "../TrajectoryGeneration/elbow_flexion_trajectory/elbow_flexion_trajectory_joint_velocity_rad.txt"

elbow_flexion_trial2 = edlut_ros_osim_path      + "/trajectory_data_test/elbow_flexion_trial2.txt"
full_elbow_flexion_trial2 = edlut_ros_osim_path + "/trajectory_data_test/full_elbow_flexion_trial2.txt"

arm26_ground_offset = edlut_ros_osim_path + "/../opensim_simple_arm_example/arm26_ground_offset.osim"
set_coordinates = edlut_ros_osim + "/../TrajectoryGeneration/MOT/setCoordinates.txt"

network_1_joint = edlut_ros_osim_path + "/elbow_flexion_trajectory/Network_1_joint_10MF_per_RBF_4MF_per_GrC_100DCN.cfg"
millard_osim = edlut_ros_osim_path + "/../models/1dof_Millard.osim"
spinal_graph = edlut_ros_osim_path + "/../spinal_cord/src/models/net.graphml"
spinal_model = edlut_ros_osim_path + "/../spinal_cord/src/models/spinal_model_net"
