#!/usr/bin/env python

""" Test file to launch opensim model. """

from osim_python.opensim_environment import OsimModel
from osim_python import file_locations


def main():

    osim_model = file_locations.millard_osim

    model = OsimModel(osim_model, 0.01, 0.0001, visualize=True)
    muscles = model.model.getMuscles()
    n_muscles = muscles.getSize()

    #: Initialize
    model.reset()
    #: Actuate and integrate 1 step
    model.actuate(np.zeros(n_muscles))
    model.integrate()


if __name__ == '__main__':
    main()
