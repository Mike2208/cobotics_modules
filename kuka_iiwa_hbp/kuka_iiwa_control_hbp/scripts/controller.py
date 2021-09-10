#!/usr/bin/env python  
import roslib
import rospy
import rospkg

from os.path import join
import sys
import warnings

import numpy as np
import scipy as sp
import scipy.linalg
import quaternion as quat
from math import pi

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from kuka_iiwa_control_hbp.msg import DesiredTrajectory, EndEffectorError

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

from dynamic_reconfigure.server import Server
from kuka_iiwa_control_hbp.cfg import ControllerConfig

from discrete_lowpass import Discrete_Low_Pass_VariableStep

np.set_printoptions(precision=3, suppress=True)

################################################
##############  UTILITY FUNCTIONS ##############

def vec_to_0quat(v):
    #v is supposed to be a np.array with shape (3,)
    return quat.as_quat_array(np.insert(v,0,0.))

def quat_prod(p,q):
    pq = np.quaternion()
    pq.w = p.w * q.w - (p.vec).dot(q.vec)
    pq.vec = p.w * q.vec + q.w * p.vec + np.cross(p.vec, q.vec)
    return pq

def kdl_to_mat(m):
    mat =  np.array(np.zeros((m.rows(), m.columns())))
    for i in range(m.rows()):
        for j in range(m.columns()):
            mat[i,j] = m[i,j]
    return mat

def joint_kdl_to_np(q):
    if q == None:
        return None
    return np.array([q[i] for i in range(q.rows())])

def joint_np_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl

################################################
##############        CLASSES       ############

class JState:
    def __init__(self, number_of_joints):
        """Class to store joints' state and its derivatives"""
        self.q = np.zeros(number_of_joints)
        self.qd = np.zeros(number_of_joints)
        self.qdd = np.zeros(number_of_joints)
        self.t = 0.0
        #self.t = {'q':0.0, 'qd':0.0, 'qdd':0.0}

class TSState:
    """Class to store position and orientation, and their derivatives """
    def __init__(self):
        self.x = {'lin': np.zeros(3),'ang': np.quaternion(1,0,0,0)}
        self.xd = {'lin': np.zeros(3),'ang': np.zeros(3)}
        self.xdd = {'lin': np.zeros(3),'ang': np.zeros(3)}
        self.t = 0.0
        #self.t = {'x':0.0, 'xd':0.0, 'xdd':0.0}

class ControllerComm:
    """Class to manage the communications (via ROS topics) needed by a Controller"""
    def __init__(self, js, ts_des, joint_order, subscribers_info, publishers_info):
        """Class to manage the communications (via ROS topics) needed by a Controller

        Parameters
        ----------
        js : JointState
            the variable used by a Controller to store the joints' state
        ts_des : TaskSpaceState
            the variable used by a Controller to store the desired end-effector's state
        joint_order : list
            list of strings, indicating the correct order in which the joints' state should be stored as a vector
        """

        # This trick is needed so that the subscribers' callback functions directly update
        # the js and ts_des variables, passed when instatiating an object of this class
        self.update_joint_state = self.create_update_js_fun(js, joint_order)
        self.update_des_trj = self.create_update_ts_des_function(ts_des)

        ### Subscribers
        # Joint state subscriber
        self.joint_state_sub = rospy.Subscriber(name=subscribers_info['joint_state'],data_class=JointState,callback=self.update_joint_state,queue_size=5)
        # Desired Trajectory subscriber
        self.des_trj_sub = rospy.Subscriber(name=subscribers_info['desired_trajectory'],data_class=DesiredTrajectory,callback=self.update_des_trj,queue_size=5)
        ### Publishers
        # Joint command publishers: torques, positions, or velocity for an EffortController
        self.joint_cmd_pub = rospy.Publisher(name=publishers_info['joint_command'],data_class=Float64MultiArray,queue_size=10)
        self.joint_cmd_msg = Float64MultiArray()
        self.joint_cmd_msg.layout.dim.append(MultiArrayDimension(label='joint_cmd',size=len(joint_order),stride=1))
        # EE error publisher
        self.EE_err_pub = rospy.Publisher(name=publishers_info['ee_error'],data_class=EndEffectorError,queue_size=5)
        self.EE_err_msg = EndEffectorError()
        # EE Path publisher, for RVIZ visualization
        self.EE_path_pub = rospy.Publisher(name='/iiwa/ee_path',data_class=Path,queue_size=5)
        self.EE_path_msg = Path()
        self.EE_path_msg.header.frame_id = 'iiwa_link_0'
    
    def create_update_js_fun(self, js, joint_order):
        def update_joint_state(joint_state):
            state_dict = dict(zip(joint_state.name, zip(joint_state.position, joint_state.velocity, joint_state.effort)))
            for index, joint in enumerate(joint_order):
                js.q[index]  = state_dict[joint][0] # position
                js.qd[index] = state_dict[joint][1] # velocity
            js.t = float(joint_state.header.stamp.secs) + float(joint_state.header.stamp.nsecs * 1.0e-9)
        return update_joint_state
    
    def create_update_ts_des_function(self, ts_des):
        def update_des_trj(des_traj):
            ts_des.x['ang'] = np.quaternion(
                des_traj.pose.orientation.w,
                des_traj.pose.orientation.x,
                des_traj.pose.orientation.y,
                des_traj.pose.orientation.z
            )
            ts_des.x['lin'] = np.array((des_traj.pose.position.x, des_traj.pose.position.y, des_traj.pose.position.z))

            ts_des.xd['ang'] = np.array((des_traj.twist.angular.x, des_traj.twist.angular.y, des_traj.twist.angular.z))
            ts_des.xd['lin'] = np.array((des_traj.twist.linear.x, des_traj.twist.linear.y, des_traj.twist.linear.z))
            
            ts_des.xd['ang'] = np.array((des_traj.twd.angular.x, des_traj.twd.angular.y, des_traj.twd.angular.z))
            ts_des.xd['lin'] = np.array((des_traj.twd.linear.x, des_traj.twd.linear.y, des_traj.twd.linear.z))

            ts_des.t = float(des_traj.header.stamp.secs) + float(des_traj.header.stamp.nsecs) * 1.0e-9

        return update_des_trj

    def publish_joint_cmd(self, cmd):     
        self.joint_cmd_msg.data = list(cmd)
        self.joint_cmd_pub.publish(self.joint_cmd_msg)
    
    def publish_EE_err(self, ex):
        ### Fill message fields
        # Time
        self.EE_err_msg.time.data = ex.t
        # Pose
        self.EE_err_msg.e.angular.x = ex.x['ang'].x
        self.EE_err_msg.e.angular.y = ex.x['ang'].y
        self.EE_err_msg.e.angular.z = ex.x['ang'].z
        self.EE_err_msg.e_ang_norm = sp.linalg.norm(ex.x['ang'].vec)
        self.EE_err_msg.e.linear.x = ex.x['lin'][0]
        self.EE_err_msg.e.linear.y = ex.x['lin'][1]
        self.EE_err_msg.e.linear.z = ex.x['lin'][2]
        self.EE_err_msg.e_lin_norm = sp.linalg.norm(ex.x['lin'])
        # Twist
        self.EE_err_msg.ed.angular.x = ex.xd['ang'][0]
        self.EE_err_msg.ed.angular.y = ex.xd['ang'][1]
        self.EE_err_msg.ed.angular.z = ex.xd['ang'][2]
        self.EE_err_msg.ed_ang_norm = sp.linalg.norm(ex.xd['ang'])
        self.EE_err_msg.ed.linear.x = ex.xd['lin'][0]
        self.EE_err_msg.ed.linear.y = ex.xd['lin'][1]
        self.EE_err_msg.ed.linear.z = ex.xd['lin'][2]
        self.EE_err_msg.ed_lin_norm = sp.linalg.norm(ex.xd['lin'])
        # publish message
        self.EE_err_pub.publish(self.EE_err_msg)

    def publish_ee_path(self, xs):
        # Update and publish path for visualization on RVIZ
        pose = PoseStamped()
        pose.pose.position.x = xs.x['lin'][0]
        pose.pose.position.y = xs.x['lin'][1]
        pose.pose.position.z = xs.x['lin'][2]
        or_tmp = quat.as_float_array(xs.x['ang'])
        pose.pose.orientation.w = or_tmp[0]
        pose.pose.orientation.x = or_tmp[1]
        pose.pose.orientation.y = or_tmp[2]
        pose.pose.orientation.z = or_tmp[3]
        self.EE_path_msg.header.stamp = rospy.Time.now()
        # self.EE_path_msg.header.frame_id = "navigation"
        self.EE_path_msg.poses.append(pose)
        self.EE_path_pub.publish(self.EE_path_msg)

class Controller:
    def __init__(self, urdf, world_link, base_link, end_link, subscribers_info, publishers_info):
        """Constructor

        Parameters
        ----------
        URDF : URDF object
            imported with urdf_parser_py.urdf, either from the parameter server or from a *.urdf file
        base_link : string
            name of the base link
        end_link : string
            name of the end link (usually the one corresponding to end-effector). The controller will control the position of this link.
        """
        ### Create KDL model
        self.urdf = urdf
        _, self.tree = treeFromUrdfModel(urdf)
        print('\nJoints: ', self.tree.getNrOfJoints())
        print('Segments: ', self.tree.getNrOfSegments())
        self.base_link = base_link
        self.end_link = end_link
        self.ee_chain = self.tree.getChain(base_link, end_link)
        self.w_chain = self.tree.getChain(world_link, base_link)
        # Initialize solvers for forward kinematics, jacobian and dynamic matrix (Mass, Coriolis, Gravity)
        self._fk_p_kdl = kdl.ChainFkSolverPos_recursive(self.ee_chain)
        self._fk_p_kdl_world = kdl.ChainFkSolverPos_recursive(self.w_chain)
        self._fk_v_kdl = kdl.ChainFkSolverVel_recursive(self.ee_chain)
        self._jac_kdl = kdl.ChainJntToJacSolver(self.ee_chain)
        # TODO: gravity rotation
        self._dyn_kdl = kdl.ChainDynParam(self.ee_chain, kdl.Vector(0, 0, -9.81)) # gravity w.r.t. the base link of the chain
        ### NOTE: joint_order refer only to the kinematic chain (e.g. it maty not include the gripper's joints, depepdning on the given end_link)
        self.joint_order = self.get_joint_names()
        self.NQ = len(self.joint_order)
        ### States
        self.js = JState(self.NQ)
        self.js_des = JState(self.NQ)
        self.eq = JState(self.NQ)
        self.xs = TSState()
        self.xs_des = TSState()
        self.ex = TSState()
        # Command to be applied to the joints
        self.joint_cmd = {'q': np.zeros(self.NQ), 't':0.0}
        # End-effector's Jacobian
        self.J = {
            'lin' : np.zeros((3,self.NQ)),
            'ang' : np.zeros((3,self.NQ))
        }
        ### Communications with topics
        self.comm = ControllerComm(self.js, self.xs_des, self.joint_order, subscribers_info, publishers_info)
        ### Controller's parameters
        self.exp_ik_damp = 5.0
        self.K_IK_2 = 0.0
        self.K_IK  = 0.0
        self.K_CTp = 0.0
        self.K_CTd = 0.0
        self.dyn_srv = Server(ControllerConfig, self.dyn_reconfigure_callback)
        ### Joints's speed low_pass
        self.js_des_lp = Discrete_Low_Pass_VariableStep(dim=self.NQ, fc=20.0) # Note: to avoid aliasing, choose fc<HZ/2, with HZ the node frequency
    
    def get_link_names(self, joints=False, fixed=True):
        """Return a list of link names in the kinematic chain"""
        return self.urdf.get_chain(self.base_link, self.end_link, joints=joints, fixed=fixed)

    def get_joint_names(self, links=False, fixed=False):
        """Return a list of link names in the kinematic chain"""
        return self.urdf.get_chain(self.base_link, self.end_link,
                                   links=links, fixed=fixed)
    
    def dyn_reconfigure_callback(self, config, level):
        self.exp_ik_damp = config['exp_ik_damp']
        self.K_IK = {
            'lin' : config['K_IK_lin']*np.eye(3),
            'ang' : config['K_IK_ang']*np.eye(3)
        }
        self.K_IK_2 = config['K_IK_2']*np.eye(self.NQ)
        self.K_CTp = config['K_CTp']*np.eye(self.NQ)
        self.K_CTd = config['K_CTd']*np.eye(self.NQ)
        return config

    def bigger_t(self, s1, s2):
        """Utility function, to properly manage time"""
        if s1.t >= s2.t:
            return s1.t
        else:
            return s2.t
    
    def __update_joint_error(self):
        self.eq.q = self.js_des.q - self.js.q
        self.eq.qd = self.js_des.qd - self.js.qd
        self.eq.t = self.bigger_t(self.js, self.js_des)
    
    def __update_ee_error(self):
        # NOTE: the orientation error is a quaternion
        self.ex.x['ang'] = (self.xs_des.x['ang']) * ( (self.xs.x['ang']).conjugate() )
        self.ex.x['lin'] = self.xs_des.x['lin'] - self.xs.x['lin']
        self.ex.xd['ang'] = self.xs_des.xd['ang'] - self.xs.xd['ang']
        self.ex.xd['lin'] = self.xs_des.xd['lin'] - self.xs.xd['lin']
        self.ex.t   = self.bigger_t(self.xs_des, self.xs)
    
    def __update_jacobian(self):
        J_kdl = kdl.Jacobian(self.NQ)
        self._jac_kdl.JntToJac(joint_np_to_kdl(self.js.q), J_kdl)
        J = kdl_to_mat(J_kdl)
        self.J['lin'] = J[0:3]
        self.J['ang'] = J[3:6]
    
    def __do_kdl_fk_p(self, q, link_number=-1):
        """Forward kinematics. Compute the roto-traslation matrix of a given link (relative to it's origin)

        Parameters
        ----------
        q : np.array or list
            joints' positions
        link_number : int, optional
            number of the link of the self.ee_chain for which we want the FK, by default -1

        Returns
        -------
        [type]
            [description]
        """
        endeffec_frame = kdl.Frame()
        kinematics_status = self._fk_p_kdl.JntToCart(joint_np_to_kdl(q),
                                                   endeffec_frame,
                                                   link_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.array([[M[0,0], M[0,1], M[0,2], p.x()], 
                             [M[1,0], M[1,1], M[1,2], p.y()], 
                             [M[2,0], M[2,1], M[2,2], p.z()],
                             [     0,      0,      0,     1]])
        else:
            return None

    def __update_ee_state(self):
        """Update end-effector state based on the stored value of the joint state and the Jacobian."""
        Tee = self.__do_kdl_fk_p(self.js.q)
        # Rotation matrix from end-effector's to world's reference frame (v_world = Ree * v_ee)
        Ree = Tee[:3, :3]
        # Update state
        self.xs.x['lin'] = Tee[:3,3]
        self.xs.x['ang'] = quat.from_rotation_matrix(Ree)

        self.xs.xd['lin'] = self.J['lin'].dot(self.js.q)
        self.xs.xd['ang'] = self.J['ang'].dot(self.js.q)

        self.xs.t = self.js.t
        
    def exp_damped_svd(self, Jmat):
        shape = Jmat.shape
        U,s,Vh = sp.linalg.svd(Jmat)
        #S = np.zeros(shape)
        Sinv = np.zeros(shape[::-1])
        Sexp = np.zeros((shape[0],shape[0]))
        for i in range(len(s)):
            #S[i,i] = s[i]
            Sinv[i,i] = 1.0/(s[i]) #a small value can be added here; this doesn't modify the algorithm but avoids a division by an (exact) zero and getting an "inf" from NumPy
            Sexp[i,i] = np.exp(-s[i]*s[i]*self.exp_ik_damp)
        return U, Sinv, Sexp, Vh

    def __update_ik_matrices(self, hierarchial=False):
        """Update the matrices used by the inverse kinematics algorithms"""
        # # Don't execute if self.js_des is already updated with the most recent values
        # if self.js.t == self.bigger_t(self.x_des, self.q):
        #     return
        # Update the desired valure of q_des and qd_des based on the current J, ex, xd_des
        ### Jacobian pseudo-inverse and exp-damped pseudo-inverse
        Jmat = np.vstack((self.J['lin'], self.J['ang']))
        U, Sinv, Sexp, Vh = self.exp_damped_svd(Jmat)
        J_expinv = Vh.T.dot( Sinv.dot(( np.eye(6) - Sexp ).dot( U.T ) ) )
        self.J_expinv = {
            'lin': J_expinv[:,0:3],
            'ang': J_expinv[:,3:6]
        }
        J_pinv = Vh.T.dot(Sinv.dot(U.T))
        self.J_pinv = {
            'lin': J_pinv[:,0:3],
            'ang': J_pinv[:,3:6]
        }
        if hierarchial:
            # For hierarchial IK, compute also the pseudo-inverse of the jacobian associated to the last 3 joints
            J_l3 = np.vstack((self.J['lin'][:,(self.NQ-3):self.NQ], self.J['ang'][:,(self.NQ-3):self.NQ]))
            U_l3, Sinv_l3, Sexp_l3, Vh_l3 = self.exp_damped_svd(J_l3)
            J_expinv_l3 = Vh_l3.T.dot( Sinv_l3.dot(( np.eye(6) - Sexp_l3 ).dot( U_l3.T ) ) )
            self.J_expinv_l3 = {
                'lin': J_expinv_l3[:,0:3],
                'ang': J_expinv_l3[:,3:6]
            }
            J_pinv_l3 = Vh.T.dot(Sinv.dot(U.T))
            self.J_pinv_l3 = {
                'lin': J_pinv_l3[:,0:3],
                'ang': J_pinv_l3[:,3:6]
            }
        # Null-space projectors
        self.P_lin = np.eye(self.NQ) - self.J_pinv['lin'].dot(self.J['lin'])
        self.P_ang = np.eye(self.NQ) - self.J_pinv['ang'].dot(self.J['ang'])
        self.P = np.eye(self.NQ) - J_pinv.dot(Jmat)
    
    def __exp_damped_IK(self):
        """Call update_ik_matrices() first, to update the matrices used by this function"""
        # As a secondary target, minimize joint displacement from their neutral position
        qd0_des = -self.K_IK_2.dot(self.js.q)
        ### Compute desired joints' speeds
        J_expinv = np.hstack((self.J_expinv['lin'],self.J_expinv['ang'])) # it's an inverse, dimensions are transposed w.r.t. the Jacobian
        xd_des = np.concatenate((self.xs_des.xd['lin'], self.xs_des.xd['ang']),axis=0)
        K_e = np.concatenate((
                (self.K_IK['lin']).dot(self.ex.x['lin']),
                (self.K_IK['ang']).dot(self.ex.x['ang'].vec)
            ),
            axis=0
        )
        #
        qd_track = J_expinv.dot(xd_des + K_e)
        qd_0 = self.P.dot(qd0_des)

        qd_des = qd_track + qd_0
        return qd_des

    def __exp_damped_hierarchical_IK(self):
        """Call update_ik_matrices(hierarchial=True) first, to update the matrices used by this function"""
        # As a secondary target, minimize joint displacement from their neutral position
        qd0_des = -self.K_IK_2.dot(self.js.q)
        ### Compute desired joints' speeds
        # position
        qd_lin = self.J_expinv['lin'].dot(self.xs_des.xd['lin'] + self.K_IK['lin'].dot(self.ex.x['lin']))
        # orientation, using only the last 3 joints and not affecting the position (projecting in its null-space)
        w_des = self.xs_des.xd['ang']-self.J['ang'].dot(qd_lin)
        qd_ang = self.P_lin[:, self.NQ-3 : self.NQ].dot((self.J_expinv_l3['ang']).dot(w_des + (self.K_IK['ang']).dot(self.ex.x['ang'].vec)))
        qd_0 = self.P.dot(qd0_des)

        qd_des = qd_lin + qd_ang + qd_0
        return qd_des
    
    def __step_desired_joints_state(self, qd_des, dt_step):
        # time
        t_des_old = self.js_des.t
        self.js_des.t = self.bigger_t(self.xs_des, self.ex)
        dt_des = self.js_des.t - t_des_old
        if dt_des<1.0e-4:
            return
        # Velocities, from IK, and accelerations as their filtered derivative
        self.js_des.qd, self.js_des.qdd = self.js_des_lp.filter(dt=dt_des, signal=qd_des)
        # # For unfiltered version, uncomment next two lines
        # self.js_des.qdd = (qd_des - self.js_des.qd)/dt 
        # self.js_des.qd = qd_des.copy()
        # Positions, forward Euler
        self.js_des.q = self.js_des.q + dt_step * self.js_des.qd

    def step_IK(self, dt, hierarchial=False):
        """Call update() prior to this, to update the values used to compute the desired joints pos/vel/acc"""
        self.__update_ik_matrices(hierarchial=hierarchial)
        if not(hierarchial):
            qd_des = self.__exp_damped_IK()
        else:
            qd_des = self.__exp_damped_hierarchical_IK()
        self.__step_desired_joints_state(qd_des, dt)
        
    def update_state(self):
        """Usually to be called before running inverse kinematics or computed torque control"""
        self.__update_jacobian()
        self.__update_joint_error()
        self.__update_ee_state()
        self.__update_ee_error()
    
    def __update_dynamics_mats(self):
        H_kdl = kdl.JntSpaceInertiaMatrix(self.NQ)
        c_kdl = kdl.JntArray(self.NQ)
        g_kdl = kdl.JntArray(self.NQ)
        # aaaaaaaaaaaaaaaaaaaaaaaaaaaaaah chaneg q and qd to self.js.q/qd
        self._dyn_kdl.JntToMass(joint_np_to_kdl(self.js.q), H_kdl)
        self._dyn_kdl.JntToCoriolis(joint_np_to_kdl(self.js.q), joint_np_to_kdl(self.js.qd), c_kdl)
        self._dyn_kdl.JntToGravity(joint_np_to_kdl(self.js.q), g_kdl)

        self.H_dyn = kdl_to_mat(H_kdl)
        self.c_dyn = joint_kdl_to_np(c_kdl)
        self.g_dyn = joint_kdl_to_np(g_kdl)
    
    def __computed_torque_control(self, stay=False):
        """Computed Torque Control. Compute the torque to apply to the joints, from the current values of the joint error and the desired joint acceleration.
        If stay=True, compute the torque to stay in the current position."""
        if stay:
            self.joint_cmd['q'] = (
                self.H_dyn.dot(self.K_CTd.dot(-self.js.qd)) +
                self.c_dyn +
                self.g_dyn
            )
        else:
            self.joint_cmd['q'] = (
                self.H_dyn.dot(self.js_des.qdd + self.K_CTd.dot(self.eq.qd) + self.K_CTp.dot(self.eq.q)) +
                self.c_dyn +
                self.g_dyn
            )
        self.joint_cmd['t'] = self.bigger_t(self.js_des, self.eq)
    
    def __backstepping_torque_control(self, stay=False):
        """BackStepping Torque Control. Compute the torque to apply to the joints, from the current values of the joint error and the desired joint acceleration.
        If stay=True, compute the torque to stay in the current position."""
        if stay:
            self.joint_cmd['q'] = (
                self.H_dyn.dot(self.K_CTd.dot(-self.js.qd)) +
                self.c_dyn +
                self.g_dyn
            )
        else:
            J_ex = self.J['lin'].T.dot(self.ex.x['lin']) + self.J['ang'].T.dot(self.ex.x['ang'].vec)
            self.joint_cmd['q'] = (
                self.H_dyn.dot(self.js_des.qdd + self.K_CTd.dot(self.eq.qd) + J_ex) + 
                self.c_dyn +
                self.g_dyn
            )
        self.joint_cmd['t'] = self.bigger_t(self.js_des, self.eq)

    def update_ctc_cmd(self, stay=False):
        self.__update_dynamics_mats()
        self.__computed_torque_control(stay)
    
    def update_bks_cmd(self, stay=False):
        self.__update_dynamics_mats()
        self.__backstepping_torque_control(stay)
    
    def update_pos_cmd(self):
        self.joint_cmd['q'] = self.js_des.q.copy()
        self.joint_cmd['t'] = self.js_des.t

class Gripper_Controller:
    def __init__(self):
        self.pos_cmd_pub = rospy.Publisher(
                    name='/iiwa/grasp_effort_controller/command',
                    data_class=Float64MultiArray,
                    queue_size=1
                )
        self.pos_cmd_msg = Float64MultiArray()
        self.pos_cmd_msg.layout.dim.append(MultiArrayDimension(label='joint_torques',size=2,stride=1))
    def publish_pos(self, pos_des):
        self.pos_cmd_msg.data = list(pos_des)
        self.pos_cmd_pub.publish(self.pos_cmd_msg)

################################################
##############        MAIN        ##############

if __name__ == '__main__':
    try:
        np.seterr(all='raise')

        NODE_NAME='controller'
        HZ = rospy.get_param('/iiwa/controller/hz', default=100)
        rospy.init_node(NODE_NAME)

        subscribers_info = {
            'joint_state' : '/iiwa/joint_states',
            'desired_trajectory' : '/iiwa/trajectory_generation/desired_ee_trajectory'
        }

        # Topics' names for the torque/position commands and the end-effector's tracking error
        # Change the line commented to use with the position controller
        publishers_info = {
            'joint_command' : '/iiwa/iiwa_effort_controller/command',
            #'joint_command' : '/iiwa/iiwa_pos_effort_controller/command',
            'ee_error' : '/iiwa/ee_error'
        }

        urdf = URDF.from_parameter_server('/robot_description')
        ct = Controller(
            urdf=urdf,
            world_link='world',
            base_link='iiwa_link_0',
            end_link='base_link_link',
            subscribers_info=subscribers_info,
            publishers_info=publishers_info
        )
        
        rate = rospy.Rate(HZ)
        dt = 1.0/float(HZ)

        while not rospy.is_shutdown():

            ct.update_state()
            ct.step_IK(dt, hierarchial=False)
            
            # Uncomment one of the two next lines to use torque control
            ct.update_ctc_cmd(stay=False)
            #ct.update_bks_cmd(stay=False)

            ### Uncomment the next line to use position control
            #ct.update_pos_cmd()

            ct.comm.publish_joint_cmd(ct.joint_cmd['q'])
            ct.comm.publish_EE_err(ct.ex)
            ct.comm.publish_ee_path(ct.xs)
            #
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    rospy.spin()

