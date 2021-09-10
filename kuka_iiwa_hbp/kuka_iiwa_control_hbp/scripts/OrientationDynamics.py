#!/usr/bin/env python 

# Copyright (C) 2021  Francesco Iori, research assistant @ Biorobotics Institute, Sant'Anna School of Advanced Studies, Pisa

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import roslib
import rospy
import numpy as np
import scipy as sp
import quaternion as quat
from geometry_msgs.msg import PoseStamped, Pose
from math import pi
import matplotlib.pyplot as plt
from discrete_lowpass import Discrete_Low_Pass_VariableStep

np.set_printoptions(precision=3)

### Utility functions for quaternion operations

def vec_to_0quat(v):
    """ Return a quaternion (0, v). Useful for vector-quaternion multiplication"""
    #v is supposed to be a np.array with shape (3,)
    return quat.as_quat_array(np.insert(v,0,0.0))

def normalize(v):
    v_norm = np.linalg.norm(v)
    if (v_norm < 1e-8):
        return np.array((0.0,0.0,0.0)) 
    else:
        return v/v_norm

def quat_prod(p,q):
    """p*q quaternion product"""
    pq = np.quaternion()
    pq.w = p.w * q.w - (p.vec).dot(q.vec)
    pq.vec = p.w * q.vec + q.w * p.vec + np.cross(p.vec, q.vec)
    return pq

def quat_exp(q):
    """exp(q) note: please refere to the mathematical definition of the log() of a quaternion"""
    # Note: returns a quaternion
    print(q.vec)
    v_norm = np.linalg.norm(q.vec)
    n = np.nan_to_num(q.vec/v_norm)
    print(n)
    print(q.w)
    vec_part =  np.exp(q.w) * n * np.sin(v_norm)
    sc_part = np.exp(q.w) * np.cos(v_norm)
    print(vec_part)
    print(sc_part)
    q_exp = np.quaternion(
        sc_part,
        vec_part[0],
        vec_part[1],
        vec_part[2]
    )
    return q_exp

def quat_log(q):
    """log(q). Note: please refere to the mathematical definition of the log() of a quaternion"""
    # Note: Returns a quaternion
    v_norm = np.linalg.norm(q.vec)
    n = np.nan_to_num(q.vec/v_norm)
    sc_part = np.log(q.norm())
    vec_part = np.nan_to_num(n * np.log(q.w / q.norm()))
    q_log = np.quaternion(
        sc_part,
        vec_part[0],
        vec_part[1],
        vec_part[2]
    )
    return q_log

def quat_pw(q,rho):
    """ Calcolate the power q^rho """
    theta = np.nan_to_num(np.linalg.norm(quat.as_rotation_vector(q.normalized())))
    n = normalize(q.vec)
    scale = (np.sqrt(q.norm()))**rho
    #theta = np.arccos(q.w)
    sc_part = scale * np.cos(theta*rho)
    vec_part = scale* n * np.sin(theta*rho)
    q_pw = np.quaternion(
        sc_part,
        vec_part[0],
        vec_part[1],
        vec_part[2]
    )
    return q_pw

###

class OrientationDynamics:
    """Use a dynamical system in the quaternion space to generate a trajectory (angular position, as a quaternion, and angular velocity)
    that converges to an input orientation goal.
    """
    def __init__(self, K, fc):
        """Create an instance of the OrientationDynamics class

        Parameters
        ----------
        K : float
            Gain
        fc : float
            cut-off frequency for the angular speed low-pass filter
        """
        self.K = K
        self.fc = fc
        self.low_pass = Discrete_Low_Pass_VariableStep(dim=3, fc=fc, K=1)
        self.state = {
            'q' : np.quaternion(1,0,0,0),
            'w' : np.zeros(3)
        }

    def set_goal(self, qg):
        """Set the goal of the system. When simulated, the system will converge to this goal."""
        self.qg = qg.copy()
    
    def step(self, q, dt):
        """Simulate one step of the dynamical system

        Parameters
        ----------
        q : np.quaternion
            quaternion describing the current orientation
        dt : float
            step duration (integration is performed with one-step forward Euler)

        Returns
        -------
        np.quaternion, np.array(3)
            orientation quaternion and angular speed after the simulated step
        """
        eq = self.qg * q.conjugate()
        w_des = self.K * eq.vec
        w, _ = self.low_pass.filter(dt, w_des)
        qd = 0.5 * vec_to_0quat(w) * q
        q = (q + qd * dt).normalized()
        return q.copy(), w
    
    def set_params(self, **kwargs):
        """ Change the values of the selected parameters"""
        for key,val in kwargs.items():
            setattr(self, key, val)
            if key == 'fc':
                filter_state_copy = self.low_pass.x.copy()
                self.low_pass = Discrete_Low_Pass_VariableStep(dim=3, fc=val, K=1)
                self.low_pass.x = filter_state_copy



# Test the code
if __name__ == '__main__':
    ### Parameters
    HZ = 240.0
    OD_params_dict = {
            'K' : 6.0,
            'fc': 8.0
        }
    dt = 1.0/HZ
    q = np.quaternion(1.0, 0.0, 0.0, 0.0)
    ### Initialize Orientation Dynamics
    od = OrientationDynamics(**OD_params_dict)
    qg = np.quaternion(*list(np.random.rand(4))) # np.quaternion(1,0,0,0) # 
    od.set_goal(qg)
    ### Simulate
    t = 0.0
    tf = 2.0
    e_array = []
    t_array = []
    w_array = []
    while (t<tf) :
        q, w = od.step(q, dt)
        #
        e_array.append((qg * q.conjugate()).vec)
        w_array.append(w)
        t_array.append(t)
        #
        t += dt
    ### Plot simulation
    e_array = np.array(e_array)
    w_array = np.array(w_array)
    t_array = np.array(t_array)
    fig_t,axs_t = plt.subplots(2,3)
    for index in range(0,3):
        axs_t[0,index].plot(
            t_array, e_array[:,index],
            color="xkcd:teal"
            )
        # axs_t[0,index].set(
        #     #ylabel="x[%d] [m]"%index,
        #     xlabel="t [s]",
        #     title='e[%d]'%index
        #     )
        axs_t[1,index].plot(
            t_array, w_array[:,index],
            color="xkcd:dark teal"
            )
        # axs_t[1,index].set(
        #     #ylabel="xd[%d] [m/s]"%index,
        #     xlabel="t [s]",
        #     title='w[%d]'%index
        #     )
    plt.show()


########
