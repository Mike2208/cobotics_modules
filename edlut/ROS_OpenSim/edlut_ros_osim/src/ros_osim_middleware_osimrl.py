#!/usr/bin/env python

""" Main file for human arm simulation in opensim controlled via a ROS node. """
import numpy as np
#import farms_pylog as pylog
from osim_python.osim_rl import OsimModel
#from osim_python import opensim_environment
import numpy as np
import time
import os
import matplotlib.pyplot as plt
#pylog.set_level('debug')

import rospy
from std_msgs.msg import Float64
import math
from decimal import Decimal

from edlut_ros_osim.msg import AnalogCompactDelay
from rosgraph_msgs.msg import Clock


class ROS_OpenSim_middleware():
    def __init__(self, joint_list, muscle_list, joint_position_topic, joint_velocity_topic, input_muscle_activation_topic, use_sim_time):
        # Save joint and muscle names
        self.joint_list = joint_list
        self.muscle_list = muscle_list

        # Arrays to store joints position and velocity
        self.joint_position = []
        self.joint_velocity = []

        # Array to store muscles activation
        self.muscle_activation = []

        # Simulation time or Real Time
        self.use_sim_time = use_sim_time

        # Initialize arrays to store joint position, velocity and muscle activation
        for joint in range(0, len(self.joint_list)):
            self.joint_position.append(0)
            self.joint_velocity.append(0)
        for muscle in range(0, len(self.muscle_list)):
            self.muscle_activation.append(0)

        self.last_state = 0.0

        # Ros publishers for joint position and velocity
        self.joint_position_publisher = rospy.Publisher(joint_position_topic, AnalogCompactDelay, queue_size = 10)
        self.joint_velocity_publisher = rospy.Publisher(joint_velocity_topic, AnalogCompactDelay, queue_size = 10)

        # Ros subscriber to receive the muscle activation to be sent to the OpenSim model
        self.muscle_activation_subscriber = rospy.Subscriber(input_muscle_activation_topic, AnalogCompactDelay, self.muscleActivationCallback)

        # Create messages to be published by the ROS publishers
        self.joint_position_msg = AnalogCompactDelay()
        self.joint_position_msg.names = self.joint_list
        self.joint_position_msg.data = [0] * len(self.joint_list)

        self.joint_velocity_msg = AnalogCompactDelay()
        self.joint_velocity_msg.names = self.joint_list
        self.joint_velocity_msg.data = [0] * len(self.joint_list)


    # Function to get the joint position and velocity and publish them in ROS Topics
    def getModelState(self, state, current_time):
        # Get the activation state for the muscles provided in the muscle_list:
        for index in range(0, len(self.muscle_list)):
            if self.muscle_list[index] in state["muscles"]:
                self.muscle_activation[index] = state["muscles"][self.muscle_list[index]]["activation"]
            else:
                rospy.loginfo("Muscle %s does not exist in the OpenSim model", self.muscle_list[index])

        # Get the position for the joints provided in the joint_list:
        for index in range(0, len(self.joint_list)):
            if self.joint_list[index] in state["joint_pos"]:
                self.joint_position[index] = state["joint_pos"][self.joint_list[index]][0]
            else:
                rospy.loginfo("Joint %s does not exist in the OpenSim model", self.joint_list[index])

        # Get the velocity for the joints provided in the joint_list:
        for index in range(0, len(self.joint_list)):
            if self.joint_list[index] in state["joint_vel"]:
                self.joint_velocity[index] = state["joint_vel"][self.joint_list[index]][0]
            else:
                rospy.loginfo("Joint %s does not exist in the OpenSim model", self.joint_list[index])

        # Publish the joints state
        self.publishJointPosition(current_time)
        self.publishJointVelocity(current_time)


    # Function to publish the joints position
    def publishJointPosition(self, time_stamp):
        # Update the message data reading from joint_position
        for joint in range(0, len(self.joint_position_msg.data)):
            self.joint_position_msg.data[joint] = self.joint_position[joint]

        # Set the message time stamp
        self.joint_position_msg.header.stamp = time_stamp

        # Publish the message
        self.joint_position_publisher.publish(self.joint_position_msg)


    # Function to publish the joints velocity
    def publishJointVelocity(self, time_stamp):
        # Update the message data reading from joint_position
        for joint in range(0, len(self.joint_velocity_msg.data)):
            self.joint_velocity_msg.data[joint] = self.joint_velocity[joint]

        # Set the message time stamp
        self.joint_velocity_msg.header.stamp = time_stamp

        # Publish the message
        self.joint_velocity_publisher.publish(self.joint_velocity_msg)


    # Callback for the muscle activation topic. Each time a message is received in the
    # muscle activation topic this function is called.
    def muscleActivationCallback(self, received_muscle_activation_msg):
        # Find the muscle name in the message and store the corresponding muscle activation
        for muscle_name in range(0, len(received_muscle_activation_msg.names)):
            found = False
            muscle_index = 0
            while (not found and muscle_index<len(self.muscle_list)):
                if received_muscle_activation_msg.names[muscle_name] == self.muscle_list[muscle_index]:
                     self.muscle_activation[muscle_index] = received_muscle_activation_msg.data[muscle_name]
                     found = True
                else:
                    muscle_index += 1


    # Function to get the muscles activation
    def getMuscleActivation(self):
        return self.muscle_activation

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

    # Joints and muscles names to be controlled
    joint_list = rospy.get_param("~joint_list")
    muscle_list = rospy.get_param("~muscle_list")

    # Ros Topics to publish joint states
    joint_position_topic = rospy.get_param("~joint_position_topic")
    joint_velocity_topic = rospy.get_param("~joint_velocity_topic")

    # Joint Topic to receive muscles activation and apply to the OpenSim model
    input_muscle_activation_topic = rospy.get_param("~input_muscle_activation_topic")

    # Sampling frequency
    sampling_frequency = rospy.get_param("~sampling_frequency")

    # Clock topic to publish time state of this node when in simulation time
    clock_topic = rospy.get_param("~clock_topic")

    # Global parameter (common to all nodes) to determine if real or simulated time is being used
    # use_sim_time = true --> simulation time in use
    use_sim_time = rospy.get_param("/use_sim_time")


    rate = rospy.Rate(500)

    # If simulated time: Subscribe to master clock and create publisher for this node's time
    if use_sim_time:
        ext_clock = ExternalClock()
        rospy.logdebug("ROS OSIM middleware subscribing to topic /clock")
        clock_subscriber = rospy.Subscriber("clock_sync", Clock, ext_clock.ClockCallback)
        time_publisher = rospy.Publisher(clock_topic, Clock, queue_size=10)
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

    rospy.loginfo("ROS-oSim middleware synchronized")

    # Create the ROS OpenSim middleware
    rosOsim = ROS_OpenSim_middleware(joint_list, muscle_list, joint_position_topic, joint_velocity_topic, input_muscle_activation_topic, use_sim_time)

    max_steps = 5000

    # Load OpenSim model
    model = OsimModel(osim_model, visualize=osim_visualization)

    #: Initialize
    model.reset()
    model.reset_manager()

    #: List model components
    model.list_elements()

    rospy.on_shutdown(rosOsim.clean_shutdown)

    while not rospy.is_shutdown():
        # If simulated time: Run next simulation time step
        if use_sim_time:
            new_time = ext_clock.GetLastConfirmed()
            # print "new_time = ", new_time.to_sec()
            if new_time > current_time:
                current_time = new_time
                # Actuate the model from the received ROS topic
                model.actuate(rosOsim.getMuscleActivation())
                # Integration musculoskeletal system
                model.integrate()
                # Update the model state after the integration
                rosOsim.getModelState(model.compute_state_desc(), current_time)
            if current_time != last_sent_time:
                 # Publish the simulation time so the synchronizer node knows when this node has simulated a time step
                 current_time_msg.clock = current_time
                 time_publisher.publish(current_time_msg)
                 last_sent_time = current_time
        # If real time: run the simulation
        else:
            current_time = rospy.get_rostime()
            # Actuate the model from the received ROS topic
            model.actuate(rosOsim.getMuscleActivation())
            # Integration musculoskeletal system
            model.integrate()
            # Update the model state after the integration
            rosOsim.getModelState(model.compute_state_desc(), current_time)
            rate.sleep()


if __name__ == '__main__':
    main()
