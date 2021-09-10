#!/usr/bin/env python

""" Main file for human arm simulation in opensim. """
import numpy as np
#import farms_pylog as pylog
from osim_python.opensim_environment import OsimModel
import numpy as np
import time
import os
import matplotlib.pyplot as plt
#pylog.set_level('debug')


def main():
    """ Main file. """

    max_steps = 5000

    #: Controller outputs
    flexor = 0.5
    extensor = 0.0

    #: osim model
    model = OsimModel(file_locations.arm26_ground_offset, 0.002, 1e-04, visualize=True)

    file = open(file_locations.set_coordinates, "r")

    joint = ["r_shoulder_elev", "r_elbow_flex"]
    joint_positions = {}
    coordinates = {}
    for j in joint:
        joint_positions[j] = []
        coordinates[j] = 20.15

    for line in file:
    	angles = line.split()
        for j in range(0, len(angles)):
			joint_positions[joint[j]].append(float(angles[j]))


    #: Initialize
    model.reset()
    model.reset_manager()

    #: List model components
    # model.list_elements()

    # for j in range(0, len(joint_positions[joint[0]])):
    #     for i in coordinates:
    #         coordinates[i] = joint_positions[i][j]
    #     model.set_coordinates(coordinates)

    #: Integrate
    for j in range(0, max_steps):
        #: Step controller here if any
        #############################
        #: Actuate the model from control outputs
        # model.actuate(
        #     [extensor, 0.0, 0.0, flexor, 0.0, 0.0]
        # )
        #: Integration musculoskeletal system
        model.set_coordinates(coordinates)
        model.integrate()
        #: Results from the integration
        # res = model.compute_state_desc()


if __name__ == '__main__':
    main()
