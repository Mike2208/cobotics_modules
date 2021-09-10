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
        print("\nExiting Mucle Activation Generator")

class MuscleActivationFromFile():
    def __init__(self, muscle_activation_file, muscle_activation_topic, muscle_list):

        self.init_time = rospy.get_rostime().to_sec()

        self.muscle_list = muscle_list

        self.muscle_activation = {}
        for muscle in self.muscle_list:
            self.muscle_activation[muscle] = []

        self.file = open(muscle_activation_file, "r")

        self.read_file()
        self.file.close()

        self.pub = rospy.Publisher(muscle_activation_topic, AnalogCompactDelay, queue_size = 10)

        self.muscle_activation_msg = AnalogCompactDelay()
        self.muscle_activation_msg.names = self.muscle_list
        self.muscle_activation_msg.data = [0] * len(self.muscle_list)

    def read_file(self):
        for line in self.file:
            activation = line.split()
            for m in range(0, len(self.muscle_list)):
                self.muscle_activation[self.muscle_list[m]].append(activation[m])

    def publishActivation(self):



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
    muscle_activation_file = rospy.get_param("~muscle_activation_file")
    muscle_activation_topic = rospy.get_param("~muscle_activation_topic")
    muscle_list = rospy.get_param("~muscle_list")
    sampling_frequency = rospy.get_param("~sampling_frequency")

    use_sim_time = rospy.get_param("/use_sim_time")

    rate = rospy.Rate(500)

    sinusoidalActivation = Sin_Generator(amplitude, frequency, phase, muscle_activation_topic, muscle_list)

    # If simulated time: Subscribe to master clock
    if use_sim_time:
        ext_clock = ExternalClock()
        clock_subscriber = rospy.Subscriber("clock", Clock, ext_clock.ClockCallback)
        current_time = rospy.Time(0.0)

    rospy.on_shutdown(sinusoidalActivation.clean_shutdown)

    while not rospy.is_shutdown():
        if use_sim_time:
            new_time = ext_clock.GetLastConfirmed()
            if new_time > current_time:
                current_time = new_time
                sinusoidalActivation.generateSinState(current_time)
                rate.sleep()

        else:
            time = rospy.get_rostime()
            sinusoidalActivation.generateSinState(time)
            rate.sleep()




if __name__ == '__main__':
    main()
