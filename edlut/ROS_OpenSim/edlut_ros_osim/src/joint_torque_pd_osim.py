#!/usr/bin/env python

##**************************************************************************
 #                      joint_torque_pd_muscle_activation.py	           *
 #                           -------------------                           *
 # copyright            : (C) 2020 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                             	   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/


# PD controller for opensim musculoskeletal model. The generated torque per joint
# has to be converted to muscle activations.

import rospy
import math

from std_msgs.msg import (
    Empty,
)

from edlut_ros_osim.msg import AnalogCompact
from edlut_ros_osim.msg import AnalogCompactDelay
from rosgraph_msgs.msg import Clock


class JointPD(object):

    def __init__(self, joint_list, kp, kd, max_torque,
                desired_position_topic, desired_velocity_topic, current_position_topic,
                current_velocity_topic, output_topic, sampling_frequency):

        self.joint_list = joint_list
        self.desired_position_topic = desired_position_topic
        self.desired_velocity_topic = desired_velocity_topic
        self.current_position_topic = current_position_topic
        self.current_velocity_topic = current_velocity_topic
        self.output_topic = output_topic

        # initialize parameters
        self.kp = dict()
        self.kd = dict()
        self.max_torque = dict()
        self.desired_position = dict()
        self.desired_velocity = dict()
        self.current_position = dict()
        self.current_velocity = dict()

        for joint in joint_list:
            self.kp[joint] = 0.0
            self.kd[joint] = 0.0
            self.max_torque[joint] = 0.0
            self.desired_position[joint] = 0.0
            self.desired_velocity[joint] = 0.0
            self.current_position[joint] = 0.0
            self.current_velocity[joint] = 0.0

        for joint in (self.kp):
            for j in range(0, len(self.joint_list)):
                if (joint == self.joint_list[j]):
                    self.kp[joint] = kp[j]
                    self.kd[joint] = kd[j]
                    self.max_torque[joint] = max_torque[j]


        self.desired_position_sub = rospy.Subscriber(self.desired_position_topic, AnalogCompactDelay, self.DesiredPositionCallback)
        self.desired_velocity_sub = rospy.Subscriber(self.desired_velocity_topic, AnalogCompactDelay, self.DesiredVelocityCallback)
        self.current_position_sub = rospy.Subscriber(self.current_position_topic, AnalogCompactDelay, self.CurrentPositionCallback)
        self.current_velocity_sub = rospy.Subscriber(self.current_velocity_topic, AnalogCompactDelay, self.CurrentVelocityCallback)
        self.output_pub = rospy.Publisher(self.output_topic, AnalogCompactDelay, queue_size=1)


        self.torque_msg = AnalogCompactDelay()
        self.torque_msg.names = self.joint_list
        self.torque_msg.data = [0] * len(self.joint_list)


    def DesiredPositionCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.desired_position.keys():
                if (x == i):
                    self.desired_position[i] = data.data[x_index]
            x_index+=1

    def CurrentPositionCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.current_position.keys():
                if (x == i):
                    self.current_position[i] = data.data[x_index]
            x_index+=1

    def DesiredVelocityCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.desired_velocity.keys():
                if (x == i):
                    self.desired_velocity[i] = data.data[x_index]
            x_index+=1

    def CurrentVelocityCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.current_velocity.keys():
                if (x == i):
                    self.current_velocity[i] = data.data[x_index]
            x_index+=1


    def UpdateTorque(self, current_time):
        for joint in self.desired_position.keys():
            x_index = 0
            for x in self.torque_msg.names:
                if (joint == x):
                    # KP portion
                    self.torque_msg.data[x_index] = self.kp[joint] * (self.desired_position[joint] - self.current_position[joint])
                    # KD portion
                    self.torque_msg.data[x_index] += self.kd[joint] * (self.desired_velocity[joint] - self.current_velocity[joint])


                    # if(self.torque_msg.data[x_index]>self.max_torque[joint]):
                    #     self.torque_msg.data[x_index]=self.max_torque[joint]
                    # if(self.torque_msg.data[x_index]<-self.max_torque[joint]):
                    #     self.torque_msg.data[x_index]=-self.max_torque[joint]

                x_index += 1

        self.torque_msg.delay = current_time.to_sec()
        self.torque_msg.header.stamp = current_time
        self.output_pub.publish(self.torque_msg)
        # print "published torque time: ", current_time.to_sec()

    def clean_shutdown(self):
        print("\nExiting torque PD control...")

class ExternalClock():
    def __init__(self):
        self.received_time = rospy.Time(0.0)
        self.first_received = False

    def ClockCallback(self, data):
        # print "PD received clock"
        self.first_received = True
        if self.received_time < data.clock:
            self.received_time = data.clock

    def GetLastConfirmed(self):
        # print "PD last confirmed: ", self.received_time
        return self.received_time

    def FirstReveived(self):
        return self.first_received


def main():
    print("Initializing node... ")
    rospy.init_node("torque_control", anonymous=True)

    #Retrieve RosLaunch parameters
    joint_list = rospy.get_param("~joint_list")
    kp = rospy.get_param("~kp")
    kd = rospy.get_param("~kd")
    max_torque = rospy.get_param("~max_torque")
    desired_position_topic = rospy.get_param("~desired_position_topic")
    desired_velocity_topic = rospy.get_param("~desired_velocity_topic")
    current_position_topic = rospy.get_param("~current_position_topic")
    current_velocity_topic = rospy.get_param("~current_velocity_topic")
    output_topic = rospy.get_param("~output_topic")
    sampling_frequency = rospy.get_param("~sampling_frequency")

    # Clock topic to publish time state of this node when in simulation time
    clock_topic = rospy.get_param("~clock_topic")

    use_sim_time = rospy.get_param("/use_sim_time")

    pd = JointPD(joint_list, kp, kd, max_torque, desired_position_topic, desired_velocity_topic, current_position_topic, current_velocity_topic,
        output_topic, sampling_frequency)

    # register shutdown callback
    rospy.on_shutdown(pd.clean_shutdown)
    control_rate = rospy.Rate(sampling_frequency)

    # If simulated time: Subscribe to master clock
    if use_sim_time:
        ext_clock = ExternalClock()
        rospy.logdebug("PD controller subscribing to topic /clock")
        clock_subscriber = rospy.Subscriber("/clock", Clock, ext_clock.ClockCallback)
        time_publisher = rospy.Publisher(clock_topic, Clock, queue_size=10)
        rospy.logdebug("PD controller publishing simulation time to topic %s", clock_topic)

        last_sent_time = rospy.Time(0.0)
        current_time = rospy.Time(0.0)
        current_time_msg = Clock()
        current_time_msg.clock = current_time

        # Publish the node init time until first time message is received from the master clock
        while not ext_clock.FirstReveived():
            rospy.logdebug("PD controller is synchronizing...")
            rospy.logdebug("PD controller publishing simulation time")
            time_publisher.publish(current_time_msg)
            # rate.sleep()

        current_time = ext_clock.GetLastConfirmed()

    while not rospy.is_shutdown():
        if use_sim_time:
            new_time = ext_clock.GetLastConfirmed()
            if new_time > current_time:
                current_time = new_time
                pd.UpdateTorque(current_time)
            if current_time != last_sent_time:
                 # Publish the simulation time so the synchronizer node knows when this node has simulated a time step
                 current_time_msg.clock = current_time
                 time_publisher.publish(current_time_msg)
                 last_sent_time = current_time
            # control_rate.sleep()
        else:
            current_time = rospy.get_rostime()
            pd.UpdateTorque(current_time)
            control_rate.sleep()


if __name__ == "__main__":
    main()
