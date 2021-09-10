#!/usr/bin/env python

""" Main file for human arm simulation in opensim controlled via a ROS node. """
import numpy as np
#import farms_pylog as pylog
# from osim_python.opensim_environment import OsimModel
from osim_python.opensim_environment_externalForce import OsimModel
from osim_python.ybot_hand_geometry import right_hand_links, right_hand_center
#from osim_python import opensim_environment
import numpy as np
import time
import os
import regex
import matplotlib.pyplot as plt
#pylog.set_level('debug')
import networkx as nx
from farms_network.neural_system import NeuralSystem
from farms_container import Container
import farms_pylog as pylog
import farms_pylog as biolog

import opensim
import rospy
from std_msgs.msg import Float64
import math
from decimal import Decimal

from edlut_ros_osim.msg import AnalogCompactDelay
from edlut_ros_osim.msg import AnalogCompact_AgonistAntagonist
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ContactsState
from scipy.optimize import minimize
import random
import spinal_cord.sc_network
#from spinal_cord.sc_circuits import SpinalCord
from edlut_ros_osim.msg import MarkerKinematicsCompact


class ROS_OpenSim_middleware():
    def __init__(self, joint_list, muscle_list, joint_position_topic, joint_velocity_topic, joint_acceleration_topic,
                 current_muscle_activation_topic, input_muscle_activation_topic, input_agonist_antagonist_topic,
                 hand_sensor_topic_list,
                 activation_exponent, use_sim_time, step_size, marker_list, marker_kinematics_topic):
        # Save joint and muscle names
        self.joint_list = joint_list
        self.muscle_list = muscle_list

        # Arrays to store joints position and velocity
        self.joint_position = []
        self.joint_velocity = []
        self.joint_acceleration = []

        self.hand_total_force = {link: np.zeros(3) for link in right_hand_links}
        self.force_time_stamp = {link: 0.0 for link in right_hand_links}

        # Array to store muscles activation
        # (current = model state activation)
        self.current_muscle_activation = []
        # (computed = muscle activation computed from desired torque)
        self.computed_muscle_activation = []
        self.previous_computed_muscle_activation = []
        # (received = muscle activation received from other module (if any))
        self.received_muscle_activation = []
        self.previous_received_muscle_activation = []

        # Excitation obtained from the computed or received muscle activations
        self.computed_excitation = []
        self.received_excitation = []

        # Dictionary of the model state
        self.model_state = {}

        # Simulation time or Real Time
        self.use_sim_time = use_sim_time

        # Activation bounds
        self.min_activation = 0.0
        self.max_activation = 1.0

        # Activation and deactivation time constants
        self.t_act = 0.01
        self.t_deact = 0.04

        self.time_step = step_size

        # Initialize data for marker kinematics
        self.marker_list = marker_list
        self.marker_position_x = len(self.marker_list)*[0]
        self.marker_position_y = len(self.marker_list)*[0]
        self.marker_position_z = len(self.marker_list)*[0]
        self.marker_velocity_x = len(self.marker_list)*[0]
        self.marker_velocity_y = len(self.marker_list)*[0]
        self.marker_velocity_z = len(self.marker_list)*[0]
        self.marker_acceleration_x = len(self.marker_list)*[0]
        self.marker_acceleration_y = len(self.marker_list)*[0]
        self.marker_acceleration_z = len(self.marker_list)*[0]


        # Initialize arrays to store joint position, velocity and muscle activation
        for joint in range(0, len(self.joint_list)):
            self.joint_position.append(0)
            self.joint_velocity.append(0)
            self.joint_acceleration.append(0)
        for muscle in range(0, len(self.muscle_list)):
            self.current_muscle_activation.append(0)
            self.computed_muscle_activation.append(0.0)
            self.previous_computed_muscle_activation.append(0.0)
            self.received_muscle_activation.append(0.0)
            self.previous_received_muscle_activation.append(0.0)
            self.computed_excitation.append(0.0)
            self.received_excitation.append(0.0)

        self.last_state = 0.0

        # Ros publishers for joint position and velocity, and current muscle activation
        self.joint_position_publisher = rospy.Publisher(joint_position_topic, AnalogCompactDelay, queue_size = 10)
        self.joint_velocity_publisher = rospy.Publisher(joint_velocity_topic, AnalogCompactDelay, queue_size = 10)
        self.joint_acceleration_publisher = rospy.Publisher(joint_acceleration_topic, AnalogCompactDelay, queue_size = 10)
        self.current_muscle_activation_publisher = rospy.Publisher(current_muscle_activation_topic, AnalogCompactDelay, queue_size = 10)
        self.computed_muscle_activation_publisher = rospy.Publisher("computed_muscle_activation", AnalogCompactDelay, queue_size = 10)
        # Publisher for the markers kinematics
        self.marker_kinematics_publisher = rospy.Publisher(marker_kinematics_topic, MarkerKinematicsCompact, queue_size = 10)

        # Ros subscriber to receive the muscle activation generated by the cerebellum
        self.received_muscle_activation_subscriber = rospy.Subscriber(input_muscle_activation_topic, AnalogCompactDelay, self.muscleActivationCallback)

        # Ros subscriber to receive the desired joint torque (i.e., generated at the cerebellum, PD controller, or other)
        self.input_agonist_antagonist_subscriber = rospy.Subscriber(input_agonist_antagonist_topic, AnalogCompact_AgonistAntagonist, self.agonistAntagonistCallback)

        # Ros subscriber to receive external forces acting on human arm
        self.hand_contact_sensor_subscriber = []
        for link in hand_sensor_topic_list:  # one subscriber per link in the human hand in gazebo
            self.hand_contact_sensor_subscriber.append(rospy.Subscriber(link, ContactsState, self.contactSensorCallback,
                                                                        callback_args=link))
        self.force_rotation_matrix = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]) \
                                     @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])  # rotation matrix for force vectors from gazebo to opensim

        # Arrays to store the cerebellar agonist and antagonist signals
        self.input_agonist = []
        self.input_antagonist = []
        for joint in self.joint_list:
            self.input_agonist.append(0.0)
            self.input_antagonist.append(0.0)
        self.input_agonist_names = self.input_antagonist_names = joint_list


        # Array to store the received torque values (following the same order as self.joint_list)
        self.desired_torque = []
        for joint in self.joint_list:
            self.desired_torque.append(0.0)
        self.desired_torque_names = joint_list

        self.activation_exponent = activation_exponent

        # Create messages to be published by the ROS publishers
        self.joint_position_msg = AnalogCompactDelay()
        self.joint_position_msg.names = self.joint_list
        self.joint_position_msg.data = [0] * len(self.joint_list)

        self.joint_velocity_msg = AnalogCompactDelay()
        self.joint_velocity_msg.names = self.joint_list
        self.joint_velocity_msg.data = [0] * len(self.joint_list)

        self.joint_acceleration_msg = AnalogCompactDelay()
        self.joint_acceleration_msg.names = self.joint_list
        self.joint_acceleration_msg.data = [0] * len(self.joint_list)

        self.current_muscle_activation_msg = AnalogCompactDelay()
        self.current_muscle_activation_msg.names = self.muscle_list
        self.current_muscle_activation_msg.data = [0] * len(self.muscle_list)

        self.computed_muscle_activation_msg = AnalogCompactDelay()
        self.computed_muscle_activation_msg.names = self.muscle_list
        self.computed_muscle_activation_msg.data = [0] * len(self.muscle_list)

        # Marker kinematics message
        self.marker_kinematics_msg = MarkerKinematicsCompact()
        self.marker_kinematics_msg.names = self.marker_list
        self.marker_kinematics_msg.position_x = len(self.marker_list)*[0]
        self.marker_kinematics_msg.position_y = len(self.marker_list)*[0]
        self.marker_kinematics_msg.position_z = len(self.marker_list)*[0]
        self.marker_kinematics_msg.velocity_x = len(self.marker_list)*[0]
        self.marker_kinematics_msg.velocity_y = len(self.marker_list)*[0]
        self.marker_kinematics_msg.velocity_z = len(self.marker_list)*[0]
        self.marker_kinematics_msg.acceleration_x = len(self.marker_list)*[0]
        self.marker_kinematics_msg.acceleration_y = len(self.marker_list)*[0]
        self.marker_kinematics_msg.acceleration_z = len(self.marker_list)*[0]

        self.muscle_activation_computed = False
        self.inputUpdate = True

    # Function to get the joint position and velocity and publish them in ROS Topics
    def getModelState(self, state, current_time):
        self.model_state = state
        # # PRINT STATE DICTIONARY
        # print "----------------"
        # for k in self.model_state:
        #     print k, " : ", self.model_state[k], "\n"
        # Get the activation state for the muscles provided in the muscle_list:
        for index in range(0, len(self.muscle_list)):
            if self.muscle_list[index] in state['muscles']:
                self.current_muscle_activation[index] = state['muscles'][self.muscle_list[index]]['activation']
            else:
                rospy.loginfo("Muscle %s does not exist in the OpenSim model", self.muscle_list[index])

        # Get the position for the joints provided in the joint_list:
        for index in range(0, len(self.joint_list)):
            if self.joint_list[index] in state['coordinate_pos']:
                self.joint_position[index] = state['coordinate_pos'][self.joint_list[index]]
            else:
                rospy.loginfo("Joint %s does not exist in the OpenSim model", self.joint_list[index])

        # Get the velocity for the joints provided in the joint_list:
        for index in range(0, len(self.joint_list)):
            if self.joint_list[index] in state['coordinate_vel']:
                self.joint_velocity[index] = state['coordinate_vel'][self.joint_list[index]]
            else:
                rospy.loginfo("Joint %s does not exist in the OpenSim model", self.joint_list[index])

        # Get the acceleration for the joints provided in the joint_list:
        for index in range(0, len(self.joint_list)):
            if self.joint_list[index] in state['coordinate_vel']:
                self.joint_acceleration[index] = state['coordinate_acc'][self.joint_list[index]]
            else:
                rospy.loginfo("Joint %s does not exist in the OpenSim model", self.joint_list[index])

        # Get the marker kinematics:
        for index in range(0, len(self.marker_list)):
            if self.marker_list[index] in state['markers']:
                self.marker_position_x[index] = state['markers'][self.marker_list[index]]['pos'][0]
                self.marker_position_y[index] = state['markers'][self.marker_list[index]]['pos'][1]
                self.marker_position_z[index] = state['markers'][self.marker_list[index]]['pos'][2]
                self.marker_velocity_x[index] = state['markers'][self.marker_list[index]]['vel'][0]
                self.marker_velocity_y[index] = state['markers'][self.marker_list[index]]['vel'][1]
                self.marker_velocity_z[index] = state['markers'][self.marker_list[index]]['vel'][2]
                self.marker_acceleration_x[index] = state['markers'][self.marker_list[index]]['acc'][0]
                self.marker_acceleration_y[index] = state['markers'][self.marker_list[index]]['acc'][1]
                self.marker_acceleration_z[index] = state['markers'][self.marker_list[index]]['acc'][2]

        # Update muscle activation
        for muscle in range(0, len(self.muscle_list)):
            self.previous_computed_muscle_activation[muscle] = state["muscles"][self.muscle_list[muscle]]["activation"]


        # Publish the joints state and muscles activation
        self.publishJointPosition(current_time)
        self.publishJointVelocity(current_time)
        self.publishJointAcceleration(current_time)
        self.publishMuscleActivation(current_time)
        # Publish the marker kinematics
        self.publishMarkerKinematics(current_time)

    # Function to publish the joints position read from the model state
    def publishJointPosition(self, time_stamp):
        # Update the message data reading from joint_position
        for joint in range(0, len(self.joint_position_msg.data)):
            self.joint_position_msg.data[joint] = self.joint_position[joint]
        # Set the message time stamp
        self.joint_position_msg.header.stamp = time_stamp
        # Publish the message
        self.joint_position_publisher.publish(self.joint_position_msg)


    # Function to publish the joints velocity read from the model state
    def publishJointVelocity(self, time_stamp):
        # Update the message data reading from joint_velocity
        for joint in range(0, len(self.joint_velocity_msg.data)):
            self.joint_velocity_msg.data[joint] = self.joint_velocity[joint]
        # Set the message time stamp
        self.joint_velocity_msg.header.stamp = time_stamp
        # Publish the message
        self.joint_velocity_publisher.publish(self.joint_velocity_msg)

    # Function to publish the joints acceleration read from the model state
    def publishJointAcceleration(self, time_stamp):
        # Update the message data reading from joint_velocity
        for joint in range(0, len(self.joint_acceleration_msg.data)):
            self.joint_acceleration_msg.data[joint] = self.joint_acceleration[joint]
        # Set the message time stamp
        self.joint_acceleration_msg.header.stamp = time_stamp
        # Publish the message
        self.joint_acceleration_publisher.publish(self.joint_acceleration_msg)

    # Function to publish the muscle activations read from the model state
    def publishMuscleActivation(self, time_stamp):
        # Update the message data reading from muscle_activation
        for muscle in range(0, len(self.current_muscle_activation_msg.data)):
            self.current_muscle_activation_msg.data[muscle] = self.current_muscle_activation[muscle]
        # Set the message time stamp
        self.current_muscle_activation_msg.header.stamp = time_stamp
        # Publish the message
        self.current_muscle_activation_publisher.publish(self.current_muscle_activation_msg)

    # Function to publish the marker kinematics read from the model state
    def publishMarkerKinematics(self, time_stamp):
        # Update the message data reading from muscle_activation
        for marker in range(0, len(self.marker_kinematics_msg.position_x)):
            self.marker_kinematics_msg.position_x[marker] = self.marker_position_x[marker]
            self.marker_kinematics_msg.position_y[marker] = self.marker_position_y[marker]
            self.marker_kinematics_msg.position_z[marker] = self.marker_position_z[marker]
            self.marker_kinematics_msg.velocity_x[marker] = self.marker_velocity_x[marker]
            self.marker_kinematics_msg.velocity_y[marker] = self.marker_velocity_y[marker]
            self.marker_kinematics_msg.velocity_z[marker] = self.marker_velocity_z[marker]
            self.marker_kinematics_msg.acceleration_x[marker] = self.marker_acceleration_x[marker]
            self.marker_kinematics_msg.acceleration_y[marker] = self.marker_acceleration_y[marker]
            self.marker_kinematics_msg.acceleration_z[marker] = self.marker_acceleration_z[marker]
        # Set the message time stamp
        self.marker_kinematics_msg.header.stamp = time_stamp
        # Publish the message
        self.marker_kinematics_publisher.publish(self.marker_kinematics_msg)

    # Callback for the muscle activation topic. Each time a message is received in the
    # muscle activation topic this function is called.
    # This can be used if muscles activations are computed from an external module; otherwise,
    # receive desired torque values and transform them to muscles activation locally (desiredTorqueCallback)
    def muscleActivationCallback(self, received_muscle_activation_msg):
        # store the previous received muscle activation (used for the activation->excitation conversion)
        for i in range(0, len(self.received_muscle_activation)):
            self.previous_received_muscle_activation[i] = self.received_muscle_activation[i]

        # Find the muscle name in the message and store the corresponding muscle activation
        for muscle_name in range(0, len(received_muscle_activation_msg.names)):
            found = False
            muscle_index = 0
            while (not found and muscle_index<len(self.muscle_list)):
                if received_muscle_activation_msg.names[muscle_name] == self.muscle_list[muscle_index]:
                     self.received_muscle_activation[muscle_index] = received_muscle_activation_msg.data[muscle_name]
                     found = True
                else:
                    muscle_index += 1

        self.receivedActivationToExcitation()

    # Callback for the contact sensor signal. forces applied to the human hand are published in 3d vectors with
    # timestamps. only total wrench is considered.
    def contactSensorCallback(self, contactsensor_msg, link_topic):

        # get timestamp and link identifier
        # time_stamp = int(str(contactsensor_msg.header.stamp))
        any((match := regex.search(link_name, link_topic)) for link_name in right_hand_links)
        link_id = match[0] if match else print("Unknown link identifier.")

        # if time_stamp > self.force_time_stamp[link_id]:
        if len(contactsensor_msg.states) > 0:
            total_force_x = contactsensor_msg.states[0].total_wrench.force.x
            total_force_y = contactsensor_msg.states[0].total_wrench.force.y
            total_force_z = contactsensor_msg.states[0].total_wrench.force.z
            self.hand_total_force[link_id] = self.force_rotation_matrix @ np.array([total_force_x, total_force_y, total_force_z])
        else:
            self.hand_total_force[link_id] = np.zeros(3)

#       print(contactsensor_msg.states[0].info)
        self.force_time_stamp[link_id] = int(str(contactsensor_msg.header.stamp))

    # Callback for the cerebellar agonist-antagonist signal.
    def agonistAntagonistCallback(self, agonist_msg):
        self.inputUpdate = False
        for joint_name in range(0, len(agonist_msg.names)):
            found = False
            joint_index = 0
            while (not found and joint_index<len(self.joint_list)):
                if agonist_msg.names[joint_name] == self.joint_list[joint_index]:
                     self.input_agonist[joint_index] = agonist_msg.agonist[joint_name]
                     self.input_antagonist[joint_index] = agonist_msg.antagonist[joint_name]
                     self.input_agonist_names[joint_index] = agonist_msg.names[joint_name]
                     found = True
                else:
                    joint_index += 1
        self.inputUpdate = True


    # Callback for the desired torque topic. Each time a message is received in the
    # desired torque topic this function is called.
    def desiredTorqueCallback(self, received_torque_msg):
        self.inputUpdate = False
        # Find the joint name in the message and store the corresponding joint torque value
        for joint_name in range(0, len(received_torque_msg.names)):
            found = False
            joint_index = 0
            while (not found and joint_index<len(self.joint_list)):
                if received_torque_msg.names[joint_name] == self.joint_list[joint_index]:
                     self.desired_torque[joint_index] = received_torque_msg.data[joint_name]
                     self.desired_torque_names[joint_index] = received_torque_msg.names[joint_name]
                     found = True
                else:
                    joint_index += 1

        rospy.logdebug("Computing Muscle Activation from Joint Torque")
        self.torqueToActivation_oneJoint(received_torque_msg.header.stamp)
        rospy.logdebug("Computing Muscle Excitation from Muscle Activation")
        self.computedActivationToExcitation()
        rospy.logdebug("Muscle Excitation computed")
        self.inputUpdate = True

    # Joint torque to muscle activation function
    def torqueToActivation_oneJoint(self, current_time):
        # # initiliaze initial guess for the optimization
        # a0 = []
        # for a in self.muscle_list:
        #     a0.append(random.uniform(self.min_activation, self.max_activation))

        # set the optimization properties: bounds, constraint function
        activation_bounds =  [(self.min_activation, self.max_activation) for a in self.muscle_list]
        # constraint1 = {'type':'eq', 'func': self.constraintFunc_oneJoint}
        # cons = ([constraint1])
        cons = ({'type':'eq', 'fun': self.constraintFunc_oneJoint})

        # find the optimized muscle activations and save them. Use previous activation as seed
        # solution = minimize(self.objectiveFunc, a0, method='SLSQP', bounds=activation_bounds, constraints=cons)
        solution = minimize(self.objectiveFunc, self.previous_computed_muscle_activation, method='SLSQP', bounds=activation_bounds, constraints=cons)
        for i in range(0, len(solution.x)):
            self.computed_muscle_activation[i] = solution.x[i]

        for a in range(0, len(self.computed_muscle_activation)):
            self.computed_muscle_activation_msg.data[a] = self.computed_muscle_activation[a]
        self.computed_muscle_activation_msg.header.stamp = current_time
        self.computed_muscle_activation_publisher.publish(self.computed_muscle_activation_msg)

        # # check result
        # muscle_index = 0
        # torque_c = 0
        # for muscle in self.muscle_list:
        #     torque_c += (self.model_state["coordinate_muscle_moment_arm"]["r_elbow_flexion"][muscle] *
        #             (self.computed_muscle_activation[muscle_index] * self.model_state["muscles"][muscle]["force_length"] * self.model_state["muscles"][muscle]["force_velocity"] - self.model_state["muscles"][muscle]["passive_force"]) *
        #             self.model_state["muscles"][muscle]["cos_pennation_angle"] * self.model_state["muscles"][muscle]["fmax"])
        #     muscle_index += 1
        #
        # print "Desired torque: ", self.desired_torque[0], "Computed torque: ", torque_c


    # Constraint function for the optimization: torque to muscle activation as described by the equation:
    # tau = "muscles_summation"[R * (a * fAL * fV  - fPE) * cos(pennation_angle) * Fmax ] + tau_residual
    # note: developed for one joint computations only (TO DO: more DOFs)
    def constraintFunc_oneJoint(self, x):
        torque_joint = self.desired_torque[0]
        joint_name = self.desired_torque_names[0]
        muscle_index = 0
        for muscle in self.muscle_list:
            torque_joint = torque_joint - (self.model_state["coordinate_muscle_moment_arm"][joint_name][muscle] *
                        (x[muscle_index] * self.model_state["muscles"][muscle]["force_length"] * self.model_state["muscles"][muscle]["force_velocity"] - self.model_state["muscles"][muscle]["passive_force"]) *
                        self.model_state["muscles"][muscle]["cos_pennation_angle"] * self.model_state["muscles"][muscle]["fmax"])
            muscle_index+=1
        return torque_joint

    # Function to be minimized:
    def objectiveFunc(self, x):
        optimize = 0.0
        for a in x:
            optimize += a**self.activation_exponent
        return optimize

    # Function to get the muscles activation
    def getMuscleActivation(self):
        return self.computed_muscle_activation

    def waitForNextMuscleActivationComputed(self):
        self.muscle_activation_computed = False

    def muscleActivationComputed(self):
        return self.muscle_activation_computed

    # Convert activation to excitation as in: https://simtk-confluence.stanford.edu/display/OpenSim/First-Order+Activation+Dynamics
    def activationToExcitation(self, activation, previous_activation):
        u_activ = []
        u_deactive = []
        excitation = []

        for a in range(0, len(self.muscle_list)):
            u_activ.append(0.0)
            u_deactive.append(0.0)
            excitation.append(0.0)

        for i in range(0, len(activation)):
            u_activ[i] = ((activation[i] - previous_activation[i]) / self.time_step ) * self.t_act*(0.5 + 1.5*activation[i]) + activation[i]
            u_deactive[i] = ((activation[i] - previous_activation[i]) / self.time_step ) * self.t_deact/(0.5 + 1.5*activation[i]) + activation[i]

        for i in range(0, len(activation)):
            if u_activ[i] > activation[i]:
                excitation[i] = max(min(u_activ[i], 1), 0)
            else:
                excitation[i] = max(min(u_deactive[i], 1), 0)


        return excitation

    def receivedActivationToExcitation(self):
        self.received_excitation = self.activationToExcitation(self.received_muscle_activation, self.previous_received_muscle_activation)

    def computedActivationToExcitation(self):
        self.computed_excitation = self.activationToExcitation(self.computed_muscle_activation, self.previous_computed_muscle_activation)

    def getReceivedExcitation(self):
        return self.received_excitation

    def getComputedExcitation(self):
        return self.computed_excitation

    def inputUpdate(self):
        return self.inputUpdate

    def clean_shutdown(self):
        print("\nExiting ROS-Osim Interface")


class ExternalClock():
    def __init__(self):
        self.received_time = rospy.Time(0.0)
        self.first_received = False

    def ClockCallback(self, data):
        self.first_received = True
        if self.received_time < data.clock:
            self.received_time = data.clock

    def GetLastConfirmed(self):
        # print "Middleware last confirmed: ", self.received_time
        return self.received_time

    def FirstReveived(self):
        return self.first_received


def main():
    # Initialize ROS node
    print("Initializing ROS-oSim Interface node... ")
    rospy.init_node("edlut_ros_osim_interface", anonymous=True, log_level=rospy.INFO)

    #
	#Retrieve parameters from roslaunch file:
    #
    # File of the OpenSim model to be used
    osim_model = rospy.get_param("~osim_model")

    # Visualization of the OpenSim model true or false
    osim_visualization = rospy.get_param("~osim_visualization")

    # Spinal Cord model file
    sc_model_file = rospy.get_param("~sc_model_file")
    net_graph_file = rospy.get_param("~net_graph_file")

    # Joints and muscles names to be controlled
    joint_list = rospy.get_param("~joint_list")
    muscle_list = rospy.get_param("~muscle_list")

    agonist_muscles = rospy.get_param("~agonist_muscles")
    antagonist_muscles = rospy.get_param("~antagonist_muscles")

    # Ros Topics to publish joint states
    joint_position_topic = rospy.get_param("~joint_position_topic")
    joint_velocity_topic = rospy.get_param("~joint_velocity_topic")
    joint_acceleration_topic = rospy.get_param("~joint_acceleration_topic")
    current_muscle_activation_topic = rospy.get_param("~current_muscle_activation_topic")

    # Joint Topic to receive muscles activation and apply to the OpenSim model
    input_muscle_activation_topic = rospy.get_param("~input_muscle_activation_topic")

    # Received cerebellar agonist and antagonist signal
    input_agonist_antagonist_topic = rospy.get_param("~input_agonist_antagonist_topic")

    # Final muscle excitation commanded to the OpenSim model
    final_muscle_excitation_topic = rospy.get_param("~final_muscle_excitation_topic")

    # Hand Contact sensor from gazebo to opensim
    hand_contact_sensor_topic = rospy.get_param("~hand_contact_sensor_topic")
    mixamorig_RightHand_sensor = rospy.get_param("~mixamorig_RightHandPalm_sensor")
    mixamorig_RightHandMiddle1_sensor = rospy.get_param("~mixamorig_RightHandMiddle1_sensor")
    mixamorig_RightHandMiddle2_sensor = rospy.get_param("~mixamorig_RightHandMiddle2_sensor")
    mixamorig_RightHandMiddle3_sensor = rospy.get_param("~mixamorig_RightHandMiddle3_sensor")
    mixamorig_RightHandIndex1_sensor = rospy.get_param("~mixamorig_RightHandIndex1_sensor")
    mixamorig_RightHandIndex2_sensor = rospy.get_param("~mixamorig_RightHandIndex2_sensor")
    mixamorig_RightHandIndex3_sensor = rospy.get_param("~mixamorig_RightHandIndex3_sensor")
    mixamorig_RightHandRing1_sensor = rospy.get_param("~mixamorig_RightHandRing1_sensor")
    mixamorig_RightHandRing2_sensor = rospy.get_param("~mixamorig_RightHandRing2_sensor")
    mixamorig_RightHandRing3_sensor = rospy.get_param("~mixamorig_RightHandRing3_sensor")
    mixamorig_RightHandPinky1_sensor = rospy.get_param("~mixamorig_RightHandPinky1_sensor")
    mixamorig_RightHandPinky2_sensor = rospy.get_param("~mixamorig_RightHandPinky2_sensor")
    mixamorig_RightHandPinky3_sensor = rospy.get_param("~mixamorig_RightHandPinky3_sensor")
    hand_sensor_topic_list = [mixamorig_RightHand_sensor, mixamorig_RightHandMiddle1_sensor,
                              mixamorig_RightHandMiddle2_sensor, mixamorig_RightHandMiddle3_sensor,
                              mixamorig_RightHandIndex1_sensor, mixamorig_RightHandIndex2_sensor,
                              mixamorig_RightHandIndex3_sensor, mixamorig_RightHandRing1_sensor,
                              mixamorig_RightHandRing2_sensor, mixamorig_RightHandRing3_sensor,
                              mixamorig_RightHandPinky1_sensor, mixamorig_RightHandPinky2_sensor,
                              mixamorig_RightHandPinky3_sensor]
    # User defined constant (P) for the minimization objective function (min SUM(muscle_activation^P))
    activation_exponent = rospy.get_param("~activation_exponent")

    # Sampling frequency
    sampling_frequency = rospy.get_param("~sampling_frequency")

    # Step size
    step_size = rospy.get_param("~step_size")

    # Max iterations
    max_iterations = rospy.get_param("~max_iterations")

    # Integrator accuracy
    integrator_accuracy = rospy.get_param("~integrator_accuracy")

    # Clock topic to publish time state of this node when in simulation time
    clock_topic = rospy.get_param("~clock_topic")

    # Topic to publish the markers kinematics
    marker_kinematics_topic = rospy.get_param("~marker_kinematics_topic")
    marker_list = rospy.get_param("~marker_list")

    # Global parameter (common to all nodes) to determine if real or simulated time is being used
    # use_sim_time = true --> simulation time in use
    use_sim_time = rospy.get_param("/use_sim_time")

    rate = rospy.Rate(500)

    # If simulated time: Subscribe to master clock and create publisher for this node's time
    if use_sim_time:
        ext_clock = ExternalClock()
        rospy.logdebug("ROS OSIM middleware subscribing to topic /clock")
        clock_subscriber = rospy.Subscriber("/clock", Clock, ext_clock.ClockCallback)
        time_publisher = rospy.Publisher(clock_topic, Clock, queue_size=10, latch=True)
        rospy.logdebug("ROS OSIM middleware publishing simulation time to topic %s", clock_topic)

        last_sent_time = rospy.Time(0.0)
        current_time = rospy.Time(0.0)
        current_time_msg = Clock()
        current_time_msg.clock = current_time

        # Publish the node init time until first time message is received from the master clock
        while not ext_clock.FirstReveived():
            rospy.logdebug("ROS OSIM middleware is synchronizing...")
            rospy.logdebug("ROS OSIM middleware publishing simulation time")
            time_publisher.publish(current_time_msg)
            # rate.sleep()

        current_time = ext_clock.GetLastConfirmed()
    else:
        current_time = rospy.get_rostime()

    rospy.loginfo("ROS-oSim middleware synchronized")

    # Publisher and message for the final muscle excitation commanded to OpenSim
    final_muscle_excitation_publisher = rospy.Publisher(final_muscle_excitation_topic, AnalogCompactDelay, queue_size = 10)
    final_muscle_excitation_msg = AnalogCompactDelay()
    final_muscle_excitation_msg.names = muscle_list
    final_muscle_excitation_msg.data = [0] * len(muscle_list)

    # # Publisher and message for the motor perturbation applied
    # perturbation_publisher = rospy.Publisher(perturbation_topic, AnalogCompactDelay, queue_size = 10)
    # perturbation_msg = AnalogCompactDelay()
    # perturbation_msg.names = perturbation_body
    # perturbation_msg.data = [0] * len(perturbation_body)

    # Create the ROS OpenSim middleware
    rosOsim = ROS_OpenSim_middleware(joint_list, muscle_list, joint_position_topic, joint_velocity_topic, joint_acceleration_topic,
                                    current_muscle_activation_topic, input_muscle_activation_topic, input_agonist_antagonist_topic,
                                     hand_sensor_topic_list,
                                    activation_exponent, use_sim_time, step_size, marker_list, marker_kinematics_topic)

    # Load OpenSim model
    # model = OsimModel(osim_model, step_size, integrator_accuracy, visualize=osim_visualization)
    model = OsimModel(osim_model, step_size, integrator_accuracy, "r_hand", visualize=osim_visualization)
    muscles = model.model.getMuscles()
    n_muscles = muscles.getSize()
    muscle_names = [None]*n_muscles
    print("xxxxxxxxxxxxxxxx")
    for i in range(n_muscles):
        muscle_names[i] = muscles.get(i).getName()
        print(muscle_names[i])
    print("xxxxxxxxxxxxxxxx")
    #: Initialize
    model.reset()
    model.reset_manager()

    # Load Spinal Cord model
    #sc_model = SpinalCord(len(muscle_list))
    #sc_model.SC_model(sc_model_file, muscle_list, step_size)
    sc_model = spinal_cord.sc_network.build_sc(sc_model_file, step_size)
    sc_model.ext_muscles.update(sc_model.flex_muscles)

    #: Initialize network
    n_steps = max_iterations*sampling_frequency
    container = Container(max_iterations=n_steps)
    net_ = NeuralSystem(
        os.path.join(
            os.path.dirname(__file__),
            net_graph_file
        ),
        container
    )
    container.initialize()
    net_.setup_integrator()

    #: List model components
    # model.list_elements()

    rospy.on_shutdown(rosOsim.clean_shutdown)

    # Excitation to be sent to the muscles
    final_excitation = []
    # Auxiliar excitation array
    excitation = []
    for i in muscle_list:
        excitation.append(0.0)
        final_excitation.append(0.0)

    #
    # MAIN PROCESSING LOOP
    #
    rosOsim.getModelState(model.get_state_dict(), current_time)
    integration_step = 0

    # Spinal cord input control signal
    controls = []
    for m in antagonist_muscles:
        controls.append(0.0)
    for m in agonist_muscles:
        controls.append(0.0)

    while not rospy.is_shutdown():
        # If simulated time: Run next simulation time step
        if use_sim_time:
            new_time = ext_clock.GetLastConfirmed()
            if new_time > current_time:
                if rosOsim.inputUpdate:
                    current_time = new_time
                    antagonist_input = rosOsim.input_antagonist[0]
                    agonist_input = rosOsim.input_agonist[0]
                    for i in range(0, len(antagonist_muscles)):
                        controls[i] = antagonist_input
                    for i in range(len(antagonist_muscles), len(antagonist_muscles)+len(agonist_muscles)):
                        controls[i] = agonist_input
                    # sc_model.update_SC_rates(integration_step, controls, model, muscle_list, step_size)
                    # model.actuate(spinal_cord.sc_circuits.muscle_excit(sc_model.mn_rates))
                    mn_rates = np.zeros(n_muscles)
                    for i in range(len(muscle_list)):
                        muscle = model.model.getMuscles().get(i)
                        container.neural.inputs.get_parameter('aff_arm_' + muscle_names[i] + '_C').value = controls[i]
                        sc_model.ext_muscles[muscle_names[i]].Prochazka_Ia_rates(model, muscle)
                        container.neural.inputs.get_parameter('aff_arm_' + muscle_names[i] + '_Ia').value = \
                            sc_model.ext_muscles[muscle_names[i]].past_Ia_rates[0]
                    net_.step(dt=step_size)
                    for i in range(n_muscles):
                        mn_rates[i] = container.neural.outputs.get_parameter('nout_arm_Mn_' + muscle_names[i]).value
                    final_excitation = mn_rates

                    for link in right_hand_links:
                        ext_force = opensim.PrescribedForce.safeDownCast(model.model.getForceSet().get(link))
                        forceFunctionSet = ext_force.get_forceFunctions()
                        funcx = opensim.Constant.safeDownCast(forceFunctionSet.get(0))  # 0 for Fx, 1 for Fy
                        funcy = opensim.Constant.safeDownCast(forceFunctionSet.get(1))  # 0 for Fx, 1 for Fy
                        funcz = opensim.Constant.safeDownCast(forceFunctionSet.get(2))  # 0 for Fx, 1 for Fy
                        funcx.setValue(rosOsim.hand_total_force[link][0])  # in N
                        funcy.setValue(rosOsim.hand_total_force[link][1])
                        funcz.setValue(rosOsim.hand_total_force[link][2])
                        #print(rosOsim.hand_total_force[link])

                    model.actuate(final_excitation)
                    # Integration musculoskeletal system
                    model.integrate()
                    # Update the model state after the integration
                    rosOsim.getModelState(model.get_state_dict(), current_time)
                    # Publish the muscle excitation commanded to OpenSim (monitoring purposes)
                    for x in range(0, len(final_excitation)):
                        final_muscle_excitation_msg.data[x] = final_excitation[x]
                    final_muscle_excitation_msg.header.stamp = current_time
                    final_muscle_excitation_publisher.publish(final_muscle_excitation_msg)
                    integration_step += 1
            if current_time != last_sent_time:
                 # Publish the simulation time so the synchronizer node knows when this node has simulated a time step
                 current_time_msg.clock = current_time
                 time_publisher.publish(current_time_msg)
                 last_sent_time = current_time
            # rospy.sleep(0.000001)
        # If real time: run the simulation
        else:
            current_time = rospy.get_rostime()
            antagonist_input = rosOsim.input_antagonist[0]
            agonist_input = rosOsim.input_agonist[0]
            for i in range(0, len(antagonist_muscles)):
                controls[i] = antagonist_input
            for i in range(len(antagonist_muscles), len(antagonist_muscles)+len(agonist_muscles)):
                controls[i] = agonist_input

            #sc_model.update_SC_rates(integration_step, controls, model, muscle_list, step_size)
            mn_rates = np.zeros(n_muscles)
            for i in range(len(muscle_list)):
                muscle = model.model.getMuscles().get(i)
                container.neural.inputs.get_parameter('aff_arm_' + muscle_names[i] + '_C').value = controls[i]
                sc_model.ext_muscles[muscle_names[i]].Prochazka_Ia_rates(model, muscle)
                container.neural.inputs.get_parameter('aff_arm_' + muscle_names[i] + '_Ia').value = \
                    sc_model.ext_muscles[muscle_names[i]].past_Ia_rates[0]
            net_.step(dt=step_size)
            for i in range(n_muscles):
                mn_rates[i] = container.neural.outputs.get_parameter('nout_arm_Mn_' + muscle_names[i]).value

            #model.actuate(spinal_cord.sc_circuits.muscle_excit(sc_model.mn_rates))
            model.actuate(mn_rates)

            # Integration musculoskeletal system
            model.integrate()
            # Update the model state after the integration
            rosOsim.getModelState(model.get_state_dict(), current_time)
            for x in range(0, len(final_excitation)):
                final_muscle_excitation_msg.data[x] = final_excitation[x]
            final_muscle_excitation_msg.header.stamp = current_time
            final_muscle_excitation_publisher.publish(final_muscle_excitation_msg)
            rate.sleep()


if __name__ == '__main__':
    main()
