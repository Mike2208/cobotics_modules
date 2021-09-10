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

import numpy as np
import scipy as sp
import scipy.integrate as integrate
import quaternion as quat
import matplotlib.pyplot as plt
from math import pi, sin , cos
import joblib as jbl

class CouplingTerm:
    """Template class to create a standard forcing term, to be used by the DMP class"""
    def __init__(self, name, type):
        """[summary]

        Parameters
        ----------
        name : string
            ID/name of the coupling term
        TAU_0 : float
            DMP's time constant
        ALPHA_X : float
            DMP's spring constant
        """
        self.name = name
        self.type = type
    
    def set_params(self, **kwargs):
        """ Change the values of the selected parameters"""
        for key,val in kwargs.items():
            setattr(self, key, val)
    
    def Ct(self):
        """
        Returns
        -------
        float
            Value to be MULTIPLIED BY, AND THEN ADDED TO, the time constant of the DMP
            (e.g. tau = tau_0 * (1 + Ct))
        """
        return 0.0
    
    def Cs(self):
        return 0.0

class GoalCoupling(CouplingTerm):
    def __init__(self, name, ALPHA_G):
        CouplingTerm.__init__(self, name, 'Goal')
        self.ALPHA_G = ALPHA_G
        self.state = { 
            'g': 0.0,
            't': 0.0
        }
        # real value (not low-pass filtered) 
        self.goal = 0.0
    
    def set_goal(self, goal):
        """Set the desired goal for this coupling term

        Parameters
        ----------
        goal : np.array
            Desired goal.
            The actual "goal" position, used to compute the DMP's virtual spring's force term, will be the filtered version of this value.
        
        Notes
        ----
        The base class' method set_params() can equivalently be used to set this term, with set_params(goal=<your goal>)
        """
        self.goal = goal
    
    def ode(self, state):
        """ODEs of the coupling term, to be used when integrating

        Parameters
        ----------
        t : float
            current time
        state : dict{'g': np.array}
            current internal goal

        Returns
        -------
        state_d, dict{'g': np.array}
            time derivatives of the corresponding keys. E.g. state_d['d'] = d('d')/dt
            #NOTE: state_d contains the DERIVATIVES of the corresponding keys.
        """
        state_d={
            'g' : self.ALPHA_G*(self.goal - state['g']),
            't' : state['t']
        }
        return state_d

    def step(self, t, dt, state, **kwargs):
        state_d = self.ode(state)
        self.state['g'] = self.state['g'] + state_d['g'] * dt
        self.state['t'] = t + dt
        return self.state.copy()
    
    def Cs(self, alpha_x, x, **kwargs):
        """Compute spatial coupling term

        Parameters
        ----------
        alpha_x : float
            alpha_x constant of the DMP
        x : np.array
            current position of the DMP
        kwargs, optional
            placeholder, so this function can be called with extra arguments (each CouplinTerms has its own self.Cs())

        Returns
        -------
        np.array
            Value to be ADDED to the right side of the DMP's transformation system (e.g. like the forcing term)
        """
        Cs = (alpha_x * alpha_x / 4.0)*(self.state['g'] - x)
        return Cs

class ForcingTerm(CouplingTerm):
    def __init__(self, name, ALPHA_F):
        """[summary]

        Parameters
        ----------
        name : string
            name of this term
        ALPHA_F : float or np.array
            low-pass filter constant
        """
        CouplingTerm.__init__(self, name, 'Forcing')
        self.ALPHA_F = ALPHA_F
        self.state = { 
            'f': 0.0,
            't': 0.0
        }
        # real value (not low-pass filtered) of the force
        self.f_ext = 0.0
        # function used to generate the external force applied to the system (i.e. self.f_ext)
        self.fnl = lambda s : 0.0
    
    def set_fnl(self, fnl):
        """Set the forcing term generator for this coupling term

        Parameters
        ----------
        fnl : callable
            Function which takes as input one argument (the phase s) and output the force to apply to the DMP system.

        Notes
        ----
        The base class' method set_params() can equivalently be used to set this term, with set_params(fnl=<your callable>)
        """
        self.fnl = fnl
     
    def ode(self, state):
        """ODEs of the coupling term, to be used when integrating

        Parameters
        ----------
        t : float
            current time
        state : dict{'g': np.array}
            current internal goal

        Returns
        -------
        state_d, dict{'g': np.array}
            time derivatives of the corresponding keys. E.g. state_d['d'] = d('d')/dt
            #NOTE: state_d contains the DERIVATIVES of the corresponding keys.
        """
        state_d={
            'f' : self.ALPHA_F*(self.f_ext - state['f']),
            't' : state['t']
        }
        return state_d

    def step(self, t, dt, state, s, **kwargs):
        self.f_ext = self.fnl(s)
        state_d = self.ode(state)
        self.state['f'] = self.state['f'] + state_d['f'] * dt
        self.state['t'] = t + dt
        return self.state.copy()
    
    def Cs(self, s, g, x0, **kwargs):
        """Compute spatial coupling term

        Parameters
        ----------
        s : float
            phase of the DMP
        g : np.array
            current goal of the DMP    
        x0 : np.array
            starting position of the DMP (when a new goal is set and the phase is reset to 1)
        kwargs, optional
            placeholder, so this function can be called with extra arguments (each CouplinTgerms has its own self.Cs())

        Returns
        -------
        np.array
            Value to be ADDED to the right side of the DMP's transformation system (e.g. like the forcing term)
        """
        # There can be more than one goal or no goal at all, in either of these cases g is passed as None
        try: 
            Cs = self.state['f'] * s * (g-x0)
            return Cs
        except TypeError:
            if g == None:
                return 0.0

class TrackingErrorCoupling:
    def __init__(self):
        pass

class Obstacle:
    def __init__(self, name, position, K, threshold):
        self.position = position
        self.name = name
        self.K = K
        self.thresh = threshold

class ObstacleAvoidanceCoupling:
    def __init__(self, obstacles):
        self.obstacles = {obs.name : obs for obs in obstacles}

    def add_obstacle(self, obstacle):
        if obstacle.name in self.obstacles.keys():
            raise UserWarning('An obstacle with the same name already exists for this ObstacleAvoidanceCoupling')
        else:
            self.obstacles[obstacle.name] = obstacle

    def change_obstacle(self, name, obstacle):
        self.obstacles[name] = obstacle
    
    def remove_obstacle(self, name):
        try:
            del self.obstacles[name]
        except KeyError:
            pass

    def Cs_Hoffman(self, obs, y, beta=6.4, gamma=1000):
        # # Hoffmann et al. 2009
        # ox = obs - y[self.X_INDEX]
        # xd = y[self.XD_INDEX]
        # theta = np.arccos((ox.T.dot(xd))/(np.linalg.norm(ox) * np.linalg.norm(xd) + 1.0e-8))
        # r = np.cross(ox, xd)
        # cs = cos(pi/4.0)
        # sn = sin(pi/4.0)
        # qr = np.quaternion(cs, sn*r[0], sn*r[1], sn*r[2])
        # xd_perp = (qr * np.quaternion(0, xd[0], xd[1], xd[2]) * qr.conjugate()).vec
        # #print(np.cross(xd_perp, xd))
        # f_obs = gamma * theta * xd_perp * np.exp(-beta * theta)
        # return f_obs
        return 0.0


class DMP_R2H:
    """
    Dynamical Movement Primitive

    A DMP is defined by the combination of two dynamical systems:\n
    1. Transformation system, which describes the evolution of the state\n
        (TAU^2)*xdd = ALPHA_X*(BETA*(G-x)-TAU*xd) + f\n
        with f a forcing term that vanish as the system moves toward its goal:
        f = fnl * s * (G-y0)\n
        and fnl a generic nonlinear term (usually learned)\n
    2. Canonical system, which describes the evolution of the timing variable:\n
        TAU*sd = -ALPHA_S*s\n
        This system can be modified, but s should monotonically converge from 1 to 0\n

    This implementation contains coupling terms for:
    - obstacle avoidance (Hoffmann et al. 2009 and also a modified better? version)
    - tracking error

    The state y is considered composed by [s, x, xd]
    s = phase
    x = position
    xd = velocity

    Notes
    -----
    Refer to Ijspeert et al. 2013 for further details on Dynamical Movement Primitves
    """
    def __init__(self, dim, TAU_0=1.0, ALPHA_X=1.0, ALPHA_S=1.0, coupling_terms=[]):
        """Dynamic Movement Primitive

        Parameters
        ----------
        dim : int
            dimension of the space (usually 3, for 3D motion)
        TAU_0 : float, optional
            DMP's time constant, by default 1.0
        ALPHA_X : float, optional
            DMP's other constant, refer to class docstring, by default 1.0
        ALPHA_S : float, optional
            DMP's phase evolution constant, by default 1.0
        coupling_terms : list, optional
            list of coupling terms to add to the DMP, by default []
        """
        self.dim = dim
        self.set_properties(TAU_0=TAU_0, ALPHA_X=ALPHA_X, ALPHA_S=ALPHA_S)
        self.y = {
            's' : 1.0,
            'x' : np.zeros(dim),
            'xd': np.zeros(dim),
            't' : 0.0
        }
        self.fnl = lambda s:0.0
        # coupling_terms is passed already as a dict for user's readibility when creating the forcing terms, but are stored internally with their internal names
        self.couplings = {cpl.name : cpl for cpl in coupling_terms}

    def set_properties(self, **kwargs):
        """ Change the values of the selected parameters
        Parameters
        ----------
        TAU_0 : float, optional
            time constant of the system
        ALPHA_X : float, optional
            refer to class docstring
        ALPHA_S : float, optional
            refer to class docstring
        ALPHA_G : float, optional
            constant for the goal's first-order low-pass filter
        fun : callable, optional
            should accept a float (the phase s, usually) as an input, e.g. fun(s)
        y : dict, optional
            DMP's state, {'s':, 'x':, 'xd':}
        """
        for key,val in kwargs.items():
            if key=='y':
                self.set_state(kwargs['y'])
                continue
            setattr(self, key, val)
 
    def add_coupling_term(self, new_cpl_term):
        """Add a coupling term to the DMP

        Parameters
        ----------
        cpl_term : CouplingTerm

        Raises
        ------
        UserWarning
            If the name of the new coupling term is the same as an already existent one (in the same DMP)
        """
        if new_cpl_term.name in self.couplings.keys():
            raise UserWarning('A coupling term with the same name already exists for this DMP')
        else:
            self.couplings[new_cpl_term] = new_cpl_term

    def set_state(self, y):
        t = self.y['t']
        if not (self.y['x'].shape[0] == y['x'].shape[0]) or (self.y['xd'].shape[0] == y['xd'].shape[0]):
            UserWarning('Error when trying to set initial state(): dimensions don\'t match')
        self.y = y.copy()
        # keep the original value of the time
        self.y['t'] = t
    
    def reset(self, s=1.0):
        self.y['s'] = s
        self.x0 = self.y['x'].copy()
    
    def get_current_goal(self):
        """Return None if there is either none or more than one goal, otherwise return the current goal"""
        num_goals = 0
        for cpl in self.couplings.values():
            if cpl.type == 'Goal':
                num_goals = num_goals + 1
                goal = cpl.state['g'].copy()
        return goal if num_goals==1 else None   

    def ode(self, y, f_ext=0.0):
        """ ODEs of the DMP. Return the time-derivative of the given input state """
        Ct = 0.0
        Cs = np.zeros(self.dim)
        # First compute TAU, because it's needed by some spatial coupling terms
        for cpl in self.couplings.values():
            Ct = Ct + cpl.Ct()
        tau = self.TAU_0 * (1 + Ct)
        if tau <= 0.0:
            raise ValueError('DMP\s ODE: negative time constant of the system')
        # THEN compute the spatial couplings
        g = self.get_current_goal()
        for cpl in self.couplings.values():
            Cs = Cs + cpl.Cs(
                alpha_x = self.ALPHA_X,
                tau = tau,
                tau_0 = self.TAU_0,
                x = y['x'],
                xd = y['xd'],
                s = y['s'],
                x0 = self.x0,
                g = g
            )
        sd = -self.ALPHA_S * y['s'] / tau
        f_eq = -self.ALPHA_X * tau * y['xd'] + Cs + f_ext
        xdd = f_eq / (tau**2)
        yd = {
            's': sd,
            'x' : y['xd'].copy(),
            'xd': xdd
        }
        return yd, f_eq

    def step(self, t, dt, y, f_ext=0.0, **kwargs):
        """ Step forward by dt, integrating once with forward Euler"""
        for cpl in self.couplings.values():
            cpl.state = cpl.step(t, dt, cpl.state, s=y['s'], x=y['x'], xd=y['xd'], **kwargs)
        yd, f_eq = self.ode(y, f_ext)
        self.y['s'] = self.y['s'] + yd['s'] * dt
        self.y['x'] = self.y['x'] + yd['x'] * dt
        self.y['xd'] = self.y['xd'] + yd['xd'] * dt
        self.y['t'] = t + dt
        return self.y.copy(), yd, f_eq
    
### Minimum Jerk
def minimum_jerk(t, tf, x0, xf):
    """Generate a minimum jerk trajectory. Useful to train a forcing term for the DMP"""
    # simple check on t and tf
    if ((t<0.0) or (tf<0.0)):
        print("Error in minimum_jerk: t or tf negative")
        raise ValueError
    t_rel = t/tf
    x   = x0 +               (xf-x0)*( 10.0*(t_rel**3) - 15.0*(t_rel**4) + 6.0*(t_rel**5) )
    xd  = (1.0/tf)*          (xf-x0)*( 30.0*(t_rel**2) - 60.0*(t_rel**3) + 30.0*(t_rel**4) )
    xdd = (1.0/tf)*(1.0/tf)* (xf-x0)*( 60.0*(t_rel)    - 180.0*(t_rel**2) + 120.0*(t_rel**3) )
    return x, xd, xdd

##################################
# Example of how to use this class, run this script to see plots with the evolution of the DMP and more
import sys
if __name__ == '__main__':
    # Create coupling terms
    cpls = {}
    cpls['Goal1'] = GoalCoupling(name='Goal1', ALPHA_G=20.0)
    cpls['Goal1'].set_goal(np.array((0.7,0.0,0.0)))

    # Create DMP
    DMP_params_dict = {
        'TAU_0':    1.0,
        'ALPHA_X':  20.0,
        'ALPHA_S':  4.0,
    }
    dmp = DMP_R2H(dim=3, coupling_terms=cpls.values(), **DMP_params_dict)
    # Set initial state
    dmp.set_properties(
        y = {
            's' : 1.0,
            'x' : np.array((-0.3,-0.5, 0.0)),
            'xd': np.zeros(3)#np.random.rand(3)-0.5,
            }
    )
    dmp.reset()

    # Create a forcing term and add it as a coupling term
    fnl = lambda s: 100.0*np.sin(np.array((0.5,0.2,0.8))*s*100.0) #0.0
    ## Load a scikit-learn model as the non-linear forcing term
    # model_name = './forcing_term_models/fnl_KNNR_minimum_jerk.joblib'
    # loaded_model = jbl.load(model_name)
    # def fnl(s):
    #     return loaded_model.predict(np.array(s).reshape(-1,1))
    forcing_cpl = ForcingTerm('Frc', ALPHA_F=100.0)
    forcing_cpl.set_fnl(fnl)
    dmp.add_coupling_term(forcing_cpl)

    # Simulate system
    dt = 1.0/300.0
    tf = 2.0
    t_array = np.arange(0.0,tf,dt)
    d_array = []
    ex_array = []
    sol = {'s':[], 'x':[], 'xd':[]}
    for t in t_array:
        # Store state, for plotting
        for key in sol.keys():
            sol[key].append(dmp.y[key])
        # Step system
        dmp.y, _, _ = dmp.step(t, dt, dmp.y)

    # Change to numpy array
    for key in sol.keys():
            sol[key] = np.array(sol[key])

    # Plot
    fig_t,axs_t = plt.subplots(2,4)
    for index in range(0,3):
        axs_t[0,index+1].plot(
            t_array, sol['x'][:,index],
            color="xkcd:teal"
            )
        axs_t[0,index+1].set(
            #ylabel="x[%d] [m]"%index,
            xlabel="t [s]",
            title='x[%d]'%index
            )
        axs_t[1,index+1].plot(
            t_array, sol['xd'][:,index],
            color="xkcd:dark teal"
            )
        axs_t[1,index+1].set(
            #ylabel="xd[%d] [m/s]"%index,
            xlabel="t [s]",
            title='xd[%d]'%index
            )
    axs_t[0,0].plot(
        t_array, sol['s'][:],
        color="xkcd:salmon"
        )
    axs_t[0,0].set(
        ylabel="s",
        xlabel="t [s]",
        title='Phase'
        )

    # Plot 2D X-Y trajectory    
    fig2, axs2 = plt.subplots(1,1)
    axs2.plot(
        sol['x'][:,0], sol['x'][:,1],
        color="xkcd:dark salmon"
        )
    axs2.set(
        ylabel="x [m]",
        xlabel="y [s]",
        title='X-Y trajectory'
        )
    plt.show()
