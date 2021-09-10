#!/usr/bin/env python

##**************************************************************************
 #                           filter_data.py                         	   *
 #                           -------------------                           *
 # copyright            : (C) 2018 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                                   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/

# This program takes the joint angles given in a file and returns a file with the
# joint velocities for that trajectory at a specified frequency.


import argparse
import struct
import sys
import numpy as np
import rospy
import math
import string
import os
from scipy.signal import savgol_filter

from math import pi, sqrt, cos, sin

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class GetVelocity(object):

	def __init__(self, file_name, trajectory_frequency):

		self.file = open(file_name, "r")
		new_file = file_name.replace(".txt", "") +"_velocity.txt"
		self.file_velocities = open(new_file, "w")

		self.num_joints = 2

		self.joints_position = []
		self.joints_velocity = []

		for joint in range(0, self.num_joints):
			self.joints_position.append([])
			self.joints_velocity.append([])


		self.read_file()

		self.trajectory_frequency = trajectory_frequency
		self.samples = len(self.joints_position[0])
		self.dt = 0.002

		self.get_velocity()

		self.write_new_file()
		self.close_files()

	def read_file(self):
		for line in self.file:
			angles = line.split()
			for joint in range(0,len(angles)):
				self.joints_position[joint].append(float(angles[joint]))

	def get_velocity(self):
		for i in range(0, self.samples):
			if i == 0 or i==1 or i==(self.samples-2) or i == (self.samples - 1):
				for joint in range(0, self.num_joints):
					self.joints_velocity[joint].append(0)
			else:
				for joint in range(0, self.num_joints):
					self.joints_velocity[joint].append((self.joints_position[joint][i-2]*(-0.2) + self.joints_position[joint][i-1]*(-0.1) + self.joints_position[joint][i+1]*(0.1) + self.joints_position[joint][i+2]*(0.2))/self.dt)

		print "DT", self.dt

	def write_new_file(self):
		for x in range(0,len(self.joints_velocity[0])):
			self.file_velocities.write(str(self.joints_velocity[0][x]) + " "+str(self.joints_velocity[1][x]) + "\n")

	def close_files(self):
		self.file.close()
		self.file_velocities.close()

def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)

	parser.add_argument(
		'-f', '--file', help=".txt file with the data to be filtered"
	)
	parser.add_argument(
		'-freq', '--frequency', type=float, help="Trajectory frequency"
	)
	args = parser.parse_args(rospy.myargv()[1:])

	vel = GetVelocity(args.file, args.frequency)
	return 0


if __name__ == '__main__':
	sys.exit(main())
