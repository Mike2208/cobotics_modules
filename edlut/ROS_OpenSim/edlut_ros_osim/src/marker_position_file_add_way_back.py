#!/usr/bin/env python

##**************************************************************************
 #                           marker_position_file_add_wayback.py       	   *
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

from osim_python import file_locations

from math import pi, sqrt, cos, sin
import matplotlib.pyplot as plt

class WriteNewFile(object):

	def __init__(self):

		self.file = open(file_locations.elbow_flexion_trial2, "r")
		self.output_file = open(file_locations.full_elbow_flexion_trial2, "w")

		self.x = []
		self.y = []
		self.z = []


		self.read_file()
		self.close_file()
		self.writeFile()

	def read_file(self):
		for line in self.file:
			positions = line.split()
			self.x.append(float(positions[0]))
			self.y.append(float(positions[1]))
			self.z.append(float(positions[2]))

	def close_file(self):
		self.file.close()

	def writeFile(self):
		for i in range(0,len(self.x)):
			self.output_file.write(str(self.x[i])+" "+str(self.y[i])+" "+str(self.z[i])+"\n")

		for i in range(1, len(self.x)+1):
			self.output_file.write(str(self.x[-i])+" "+str(self.y[-i])+" "+str(self.z[-i])+"\n")



def main():
	write = WriteNewFile()

	return 0

if __name__ == '__main__':
	sys.exit(main())
