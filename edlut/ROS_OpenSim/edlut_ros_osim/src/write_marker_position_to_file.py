#!/usr/bin/env python

##**************************************************************************
 #                           write_marker_position_to_file.py              *
 #                           -------------------                           *
 # copyright            : (C) 2020 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                        	       *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/

# This node read a topic with a marker position (X Y Z) and writes the data to a .txt file.

import rospy
from std_msgs.msg import Time
from std_msgs.msg import Bool
import numpy as np
import baxter_interface
import edlut_ros_osim.msg
from edlut_ros_osim.msg import AnalogCompactDelay
from rosgraph_msgs.msg import Clock


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

class Endpoint_record(object):

	def __init__(self, topic, file):
		self.marker_position_topic = topic
		self.output_file_name = file

		self.output_file = open(self.output_file_name, "w")

		self.marker_position_subscriber = rospy.Subscriber(self.marker_position_topic, AnalogCompactDelay, self.markerPositionCallback)

		self.x = []
		self.y = []
		self.z = []
		self.data = []
		self.time = []

		self.done = False

	def markerPositionCallback(self, marker_position_msg):
		self.x.append(marker_position_msg.data[0])
		self.y.append(marker_position_msg.data[1])
		self.z.append(marker_position_msg.data[2])

		self.time.append(marker_position_msg.header.stamp.to_sec())


	def writeFile(self):
		for i in range(0,len(self.x)):
			self.output_file.write(str(self.time[i])+" "+str(self.x[i])+" "+str(self.y[i])+" "+str(self.z[i])+"\n")


def main():
	rospy.init_node('write_marker_position_to_file', anonymous=True, disable_signals = True)

	marker_position_topic = rospy.get_param("~marker_position_topic")
	output_file_name = rospy.get_param("~output_file")

	use_sim_time = rospy.get_param("/use_sim_time")

	xyz = Endpoint_record(marker_position_topic, output_file_name)
	rate = rospy.Rate(500)

	# If simulated time: Subscribe to master clock
	if use_sim_time:
		ext_clock = ExternalClock()
		clock_subscriber = rospy.Subscriber("clock", Clock, ext_clock.ClockCallback)
		current_time = rospy.Time(0.0)

	# while not rospy.is_shutdown():
	# 	if use_sim_time:
	# 		new_time = ext_clock.GetLastConfirmed()
	# 		if new_time > current_time:
	# 			current_time = new_time
	# 			rate.sleep()
	rospy.spin()
	xyz.writeFile()
	rospy.signal_shutdown("writing done")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
