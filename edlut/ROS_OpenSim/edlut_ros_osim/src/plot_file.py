#!/usr/bin/env python

##**************************************************************************
 #                           plot_2files.py                         	   *
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


import argparse
import struct
import sys
import numpy as np
import rospy
import math
import string



from math import pi, sqrt, cos, sin
import matplotlib.pyplot as plt

class PlotFile(object):

	def __init__(self, file_name):

		self.file = open(file_name, "r")

		self.num_joints = 2
		self.first_line = False

		self.joints = []

		for joint in range(0, self.num_joints):
			self.joints.append([])

		self.read_files()
		print self.joints
		self.close_files()


	def read_files(self):
		for line in self.file:
			angles = line.split()
			for joint in range(0,len(angles)):
				self.joints[joint].append(float(angles[joint]))


	def close_files(self):
		self.file.close()


	def plot(self):
		lines = []
		for joint in range(0, self.num_joints):
			lines.append([])

		for joint in range(0, self.num_joints):
			lines[joint] = plt.plot(self.joints[joint], color="r")

		plt.legend()
		plt.show()



def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)

	parser.add_argument(
		'-f', '--file', help=".txt file with the data to be filtered"
	)

	args = parser.parse_args(rospy.myargv()[1:])

	draw = PlotFile(args.file)
	draw.plot()

	return 0


if __name__ == '__main__':
	sys.exit(main())
