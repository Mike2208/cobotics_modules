#!/usr/bin/env python

""" Main file for human arm simulation in opensim controlled via a ROS node. """
import numpy as np
#import farms_pylog as pylog
from osim_python.osim_rl import OsimModel
#from osim_python import opensim_environment
import numpy as np
import time as py_time
import os
import matplotlib.pyplot as plt
#pylog.set_level('debug')

import rospy
from std_msgs.msg import Float64
import math
from decimal import Decimal

from edlut_ros_osim.msg import AnalogCompactDelay
from edlut_ros_osim.msg import AnalogCompact_AgonistAntagonist
from rosgraph_msgs.msg import Clock


class Sin_Generator():
    def __init__(self, amplitude, frequency, phase, muscle_activation_topic, muscle_list):

        self.init_time = rospy.get_rostime().to_sec()

        self.muscle_list = muscle_list

        self.amplitude =  amplitude
        self.frequency =  frequency
        self.phase = phase

        self.pub = rospy.Publisher(muscle_activation_topic, AnalogCompactDelay, queue_size = 10)

        self.muscle_activation_msg = AnalogCompactDelay()
        self.muscle_activation_msg.names = self.muscle_list
        self.muscle_activation_msg.data = [0] * len(self.muscle_list)


    def generateSinState(self, time):
        elapsed_time = time.to_sec() - self.init_time
        for muscle in range(0,len(self.muscle_list)):
            state = self.amplitude[muscle]/2.0 +  (self.amplitude[muscle]/2.0 * math.sin(2*3.14*self.frequency[muscle]*elapsed_time + self.phase[muscle]))
            self.muscle_activation_msg.data[muscle] = state

        self.muscle_activation_msg.header.stamp = time
        self.pub.publish(self.muscle_activation_msg)

    def clean_shutdown(self):
        print("\nExiting Generator")

class Step_Generator():
    def __init__(self, value, init_step_time, duration, topic, joint_list, agonist_active, antagonist_active):

        self.init_time = init_step_time
        self.duration = duration

        self.joint_list = joint_list
        self.value =  value
        self.agonist_active = agonist_active
        self.antagonist_active = antagonist_active

        self.pub = rospy.Publisher(topic, AnalogCompact_AgonistAntagonist, queue_size = 10)

        self.msg = AnalogCompact_AgonistAntagonist()
        self.msg.names = self.joint_list
        self.msg.agonist = [0] * 3
        self.msg.antagonist = [0] * 3

    def generateStep(self, time):
        elapsed_time = time.to_sec() - self.init_time

        if time.to_sec()>=self.init_time and time.to_sec()<=self.init_time + self.duration:
            for i in range(0, len(self.joint_list)):
                self.msg.agonist[i] = self.value * self.agonist_active[i]
                self.msg.antagonist[i] = self.value * self.antagonist_active[i]
        else:
            for i in range(0, len(self.joint_list)):
                self.msg.agonist[i] = 0.0
                self.msg.antagonist[i] = 0.0


        self.msg.header.stamp = time
        self.pub.publish(self.msg)

    def clean_shutdown(self):
        print("\nExiting Generator")

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
    rospy.init_node("muscle_activation_node", anonymous=True)

    output_topic = rospy.get_param("~output_topic")
    muscle_list = rospy.get_param("~muscle_list")
    joint_list = rospy.get_param("~joint_list")
    sampling_frequency = rospy.get_param("~sampling_frequency")
    sin_generator = rospy.get_param("~sin_generator")
    amplitude = rospy.get_param("~amplitude")
    frequency = rospy.get_param("~frequency")
    phase = rospy.get_param("~phase")
    step_generator = rospy.get_param("~step_generator")
    init_step_time = rospy.get_param("~init_step_time")
    duration = rospy.get_param("~duration")
    value = rospy.get_param("~value")
    agonist_active = rospy.get_param("~agonist_active")
    antagonist_active = rospy.get_param("~antagonist_active")



    use_sim_time = rospy.get_param("/use_sim_time")

    rate = rospy.Rate(500)

    if sin_generator:
        sinusoidalActivation = Sin_Generator(amplitude, frequency, phase, output_topic, muscle_list)
        rospy.on_shutdown(sinusoidalActivation.clean_shutdown)
    elif step_generator:
        stepActivation = Step_Generator(value, init_step_time, duration, output_topic, joint_list, agonist_active, antagonist_active)
        rospy.on_shutdown(stepActivation.clean_shutdown)

    # If simulated time: Subscribe to master clock
    if use_sim_time:
        ext_clock = ExternalClock()
        clock_subscriber = rospy.Subscriber("/clock", Clock, ext_clock.ClockCallback)
        current_time = rospy.Time(0.0)


    while not rospy.is_shutdown():
        if use_sim_time:
            new_time = ext_clock.GetLastConfirmed()
            if new_time > current_time:
                current_time = new_time
                if sin_generator:
                    sinusoidalActivation.generateSinState(current_time)
                elif step_generator:
                    stepActivation.generateStep(current_time)
                # rate.sleep()
        else:
            time = rospy.get_rostime()
            sinusoidalActivation.generateSinState(time)
            rate.sleep()




if __name__ == '__main__':
    main()
