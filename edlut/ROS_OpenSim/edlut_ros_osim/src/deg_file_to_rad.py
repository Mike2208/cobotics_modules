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

from osim_python import file_locations

from scipy.signal import savgol_filter

from math import pi, sqrt, cos, sin

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class GetRad(object):

	def __init__(self):

		self.file_pos = open(file_locations.elbow_flexion_one_joint_pos, "r")
		self.file_vel = open(file_locations.elbow_flexion_one_joint_vel, "r")

		self.joint_list = ["r_shoulder_elev", "r_elbow_flexion"]

		self.new_file_pos = open(file_locations.traj_elbow_flexion_one_joint_pos, "w")
		self.new_file_vel = open(file_locations.traj_elbow_flexion_one_joint_vel, "w")

		self.new_pos = {}
		self.new_vel = {}
		for joint in self.joint_list:
			self.new_pos[joint] = []
			self.new_vel[joint] = []

		self.read_files()
		self.close_files()
		self.write_files()


	def read_files(self):
		for line in self.file_pos:
			pos = line.split()
			for joint in range(0, len(self.joint_list)):
				self.new_pos[self.joint_list[joint]].append(float(pos[joint]) * 3.141516/180.0)
		for line in self.file_vel:
			vel = line.split()
			for joint in range(0, len(self.joint_list)):
				self.new_vel[self.joint_list[joint]].append(float(vel[joint]) * 3.141516/180.0)

	def close_files(self):
		self.file_pos.close()
		self.file_vel.close()

	def write_files(self):
		for i in range(0, len(self.new_pos[self.joint_list[0]])):
			for joint in self.joint_list:
				self.new_file_pos.write(str(self.new_pos[joint][i])+" ")
				self.new_file_vel.write(str(self.new_vel[joint][i])+" ")
			self.new_file_pos.write("\n")
			self.new_file_vel.write("\n")


def main():
	rad = GetRad()
	return 0


if __name__ == '__main__':
	sys.exit(main())
