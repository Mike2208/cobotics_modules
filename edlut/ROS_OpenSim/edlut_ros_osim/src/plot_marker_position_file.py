#!/usr/bin/env python

##**************************************************************************
 #                           plot_marker_position_file.py              	   *
 #                           -------------------                           *
 # copyright            : (C) 2020 by Ignacio Abadia                       *
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

from osim_python import file_locations

from math import pi, sqrt, cos, sin
import matplotlib.pyplot as plt

class PlotFile(object):

	def __init__(self):

		self.file = open(file_locations.full_elbow_flexion_trial2, "r")

		self.x = []
		self.y = []
		self.z = []


		self.read_file()
		self.close_file()


	def read_file(self):
		for line in self.file:
			positions = line.split()
			self.x.append(float(positions[0]))
			self.y.append(float(positions[1]))
			self.z.append(float(positions[2]))

	def close_file(self):
		self.file.close()


	def plot(self):
		line_x = []
		line_y = []
		line_z = []

		plt.plot(self.x, color='r', label="x")
		plt.plot(self.y, color='b', label="y")
		plt.plot(self.z, color='g', label="z")

		plt.legend()
		plt.show()



def main():
	draw = PlotFile()
	draw.plot()

	return 0

if __name__ == '__main__':
	sys.exit(main())
