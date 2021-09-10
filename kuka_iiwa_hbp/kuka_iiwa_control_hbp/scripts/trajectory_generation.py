#!/usr/bin/env python 
import roslib
import rospy
import rospkg
import os.path
import numpy as np
import scipy as sp
import quaternion as quat
from std_msgs.msg import Float64
from kuka_iiwa_control.msg import DesiredTrajectory
from DMP_R2H import *
from OrientationDynamics import OrientationDynamics
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

from kuka_iiwa_control_hbp.cfg import TrajectoryDynamicsConfig
from dynamic_reconfigure.server import Server


class TrajectoryGenerator:
    def __init__(self, subscribers, publishers, dmp, od):

        self.dmp = dmp
        self.od = od
        
        #TODO: add type/content checks on subscribers and publishers
        self.tg_pub = rospy.Publisher(
            name=publishers['desired_trajectory'],# '/trajectory_generator/desired_ee_trajectory',
            data_class=DesiredTrajectory,
            queue_size=2
        )
        self.des_traj = DesiredTrajectory()
        self.goal_sub = rospy.Subscriber(
            name=subscribers['goal'],# '/trajectory_generator/goal',
            data_class=Pose,
            callback=self.update_goal,
            queue_size=2
        )
        self.xdes_path_pub = rospy.Publisher(
            name='/iiwa/ee_des_path',
            data_class=Path,
            queue_size=5
        )
        self.des_path_msg = Path()
        self.des_path_msg.header.frame_id = 'iiwa_link_0'
        # Dynamic reconfigure server
        self.dyn_srv = Server(TrajectoryDynamicsConfig, self.dyn_reconfigure_callback)

    def update_goal(self, msg):
        ### DMP
        G = np.array((msg.position.x, msg.position.y, msg.position.z))
        for cpl in self.dmp.couplings.values():
            if cpl.type == 'Goal':
                cpl.set_goal(G)
        # also reset DMP phase and starting position (used by the forcing tem)
        self.dmp.reset()
        ### OD
        qg = (np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)).normalized()
        self.od.set_goal(qg)
    
    def step_DMP(self, t, dt):
        self.dmp.y, self.yd, f_eq = dmp.step(t, dt, self.dmp.y)
        return self.dmp.y.copy(), f_eq
    
    def step_OD(self, dt):
        self.w_old = self.od.state['w'].copy()
        self.od.state['q'], self.od.state['w'] = self.od.step(self.od.state['q'], dt)
        # derivative of angular speed
        self.wd = (self.od.state['w'] - self.w_old)/dt
        return self.od.state['q'].copy(), self.od.state['w'].copy()
    
    def dyn_reconfigure_callback(self, config, level):
        self.dmp.set_properties(
            TAU_0 = config['dmp_TAU_0'],
            ALPHA_X = config['dmp_ALPHA_X'],
            ALPHA_S = config['dmp_ALPHA_S']
        )
        self.od.set_params(
            K = config['od_K'],
            fc = config['od_fc']
        )
        return config
    
    def publish_traj(self, t):
        ### Fill message fields
        # Time
        self.des_traj.time.data = t
        # Pose
        self.des_traj.pose.position.x = self.dmp.y['x'][0]
        self.des_traj.pose.position.y = self.dmp.y['x'][1]
        self.des_traj.pose.position.z = self.dmp.y['x'][2]
        self.des_traj.pose.orientation.w = self.od.state['q'].w
        self.des_traj.pose.orientation.x = self.od.state['q'].x
        self.des_traj.pose.orientation.y = self.od.state['q'].y
        self.des_traj.pose.orientation.z = self.od.state['q'].z
        # Twist
        self.des_traj.twist.linear.x = self.dmp.y['xd'][0]
        self.des_traj.twist.linear.y = self.dmp.y['xd'][1]
        self.des_traj.twist.linear.z = self.dmp.y['xd'][2]
        self.des_traj.twist.angular.x = self.od.state['w'][0]
        self.des_traj.twist.angular.y = self.od.state['w'][1]
        self.des_traj.twist.angular.z = self.od.state['w'][2]
        # Acceleration
        self.des_traj.twd.linear.x = self.yd['xd'][0]
        self.des_traj.twd.linear.y = self.yd['xd'][1]
        self.des_traj.twd.linear.z = self.yd['xd'][2]
        self.des_traj.twd.angular.x = self.wd[0]
        self.des_traj.twd.angular.y = self.wd[1]
        self.des_traj.twd.angular.z = self.wd[2]
        # publish message
        self.tg_pub.publish(self.des_traj)
    
    def publish_des_path(self):
        ### Publish also as path for visualization in RVIZ
        pose = PoseStamped()
        pose.pose.position.x = self.dmp.y['x'][0]
        pose.pose.position.y = self.dmp.y['x'][1]
        pose.pose.position.z = self.dmp.y['x'][2]
        or_tmp = quat.as_float_array(self.od.state['q'])
        pose.pose.orientation.w = or_tmp[0]
        pose.pose.orientation.x = or_tmp[1]
        pose.pose.orientation.y = or_tmp[2]
        pose.pose.orientation.z = or_tmp[3]
        self.des_path_msg.header.stamp = rospy.Time.now()
        # self.EE_path_msg.header.frame_id = "navigation"
        self.des_path_msg.poses.append(pose)
        self.xdes_path_pub.publish(self.des_path_msg)
        

if __name__ == '__main__':
    try:
        NODE_NAME='trajectory_generation'
        # 
        HZ = rospy.get_param('/iiwa/trajectory_generation/hz', default=100)
        rospy.init_node(NODE_NAME)
        rate = rospy.Rate(HZ)
        dt = 1.0/float(HZ)

        ### Time management (needed for the NRP)
        # Initialize internal time
        # otherwise rospy.get_time() returns 0.0, and then jumps to the current simulation time (big value) when the simulation is started
        t_curr = rospy.get_time()
        t_last = rospy.get_time()
        t = 0.0
        # rate.sleep() # NEEDED WHEN USING THE NRP, so the rest of the code starts to execute only if the simulation starts

        subscribers = {'goal': '/iiwa/trajectory_generation/goal'}
        publishers = {'desired_trajectory': '/iiwa/trajectory_generation/desired_ee_trajectory'}

        ### DMP
        # Create coupling terms
        cpls = {}
        cpls['Goal1'] = GoalCoupling(name='Goal1', ALPHA_G=20.0)
        # Set a goal, this can then be updated via the correct ROS topic
        cpls['Goal1'].set_goal(np.array((0.0,0.0,1.306))) # value of the kuka iiwa end-effector's position when imported with all joints set to 0.0
        # Create DMP
        # NOTE: these values will be overwritten by the default ones in the TrajectoryDynamics.cfg file
        DMP_params_dict = {
            'TAU_0':    1.0,
            'ALPHA_X':  20.0,
            'ALPHA_S':  4.0,
        }
        dmp = DMP_R2H(dim=3, coupling_terms=cpls.values(), **DMP_params_dict)
        # Set initial state to the current end-effector's position
        dmp.set_properties(
            y = {
                's' : 1.0,
                'x' : np.array((0.0, 0.0, 1.306)),
                'xd': np.zeros(3)
                }
        )
        dmp.reset()

        # Create a forcing term and add it as a coupling term

        # # Load a scikit-learn model as the non-linear forcing term
        # rpkg = rospkg.RosPack()
        # package_path = rpkg.get_path('kuka_iiwa_control_hbp')
        # model_name = 'fnl_KNNR_minimum_jerk.joblib'
        # model_path = os.path.join(package_path, 'forcing_term_models', model_name)        
        # loaded_model = jbl.load(model_path)
        # def fnl(s):
        #     return loaded_model.predict(np.array(s).reshape(-1,1))

        fnl = lambda s: 0.0

        forcing_cpl = ForcingTerm('Frc1', ALPHA_F=(HZ/2.0))
        forcing_cpl.set_fnl(fnl)
        dmp.add_coupling_term(forcing_cpl)

        ### Orientation Dynamics
        # NOTE: these values will be overwritten by the default ones in the TrajectoryDynamics.cfg file
        OD_params_dict = {
            'K' : 4.0,
            'fc': 7.0
        }
        dt = 1.0/HZ
        ### Initialize Orientation Dynamics
        od = OrientationDynamics(**OD_params_dict)
        od.q = np.quaternion(1,0,0,0) #Starting orientation of the Kuka, with all l9inks set to 0.0
        qg = np.quaternion(1,0,0,0) # np.quaternion(*list(np.random.rand(4))) # 
        od.set_goal(qg)

        ### Initialize Trajectorygenerator
        tg = TrajectoryGenerator(subscribers,publishers, dmp, od)

        ### Execute trajectory
        while not rospy.is_shutdown():
            # Manage time
            t_curr = rospy.get_time()
            dt_last = t_curr - t_last
            t += dt_last
            t_last = t_curr
            ###

            y, f_eq = tg.step_DMP(t, dt)
            o, w = tg.step_OD(dt)

            tg.publish_traj(t)
            tg.publish_des_path()

            rate.sleep()
    #
    except rospy.ROSInterruptException:
        pass
    rospy.spin()