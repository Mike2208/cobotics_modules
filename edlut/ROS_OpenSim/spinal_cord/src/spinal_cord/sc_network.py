import networkx as nx
import os
from matplotlib import pyplot as plt
from farms_network.neural_system import NeuralSystem
from farms_container import Container
import numpy as np
import farms_pylog as pylog
import farms_pylog as biolog


class Muscle(object):
    """ Muscle properties """

    def __init__(self, name, step_size, type=None, Ia_antagonists=None, Ia_delay=0, Ia_w=1, IaIn_w=[1], IaIn_IaIn_w=1,
                 MN_RC_w=1, RC_MN_w=1, RC_IaIn_w=1, RC_RC_w=1, past_fiber_l=None):
        self.name = name
        self.type = type
        self.Ia_delay = Ia_delay
        self.Ia_antagonists = Ia_antagonists
        self.Ia_w = Ia_w
        if Ia_antagonists is not None and len(IaIn_w) == 1:
            self.IaIn_w = IaIn_w*np.ones(len(Ia_antagonists))
        else:
            self.IaIn_w = IaIn_w
        self.IaIn_IaIn_w = IaIn_IaIn_w
        self.MN_RC_w = MN_RC_w
        self.RC_MN_w = RC_MN_w
        self.RC_IaIn_w = RC_IaIn_w
        self.RC_RC_w = RC_RC_w
        self.past_fiber_l = past_fiber_l
        self.past_Ia_rates = np.zeros(max(1, int(self.Ia_delay / (step_size * 1000))))


    def Prochazka_Ia_rates(self, model, muscle, a=4.3, b=2, c=10):
        """ Compute Prochazka Ia rates """
        opt_l = muscle.getOptimalFiberLength() * 1000
        max_v = muscle.getMaxContractionVelocity() * opt_l
        fiber_l = muscle.getFiberLength(model.state) * 1000
        if self.past_fiber_l is None:
            self.past_fiber_l = opt_l
            fiber_v = 0.001
        else:
            fiber_v = (fiber_l - self.past_fiber_l) / model.step_size
        self.past_fiber_l = fiber_l
        rate = a * np.sign(fiber_v) * np.exp(0.6 * np.log(max(min(abs(fiber_v), max_v), 0.01))) + \
               b * (min(fiber_l, 1.5 * opt_l) - opt_l) + c
        norm_rate = max(rate / (a * np.exp(0.6 * np.log(max_v)) + b * 0.5 * opt_l + c), 0)

        #: Update past rates
        self.past_Ia_rates[0:-1] = self.past_Ia_rates[1:]
        self.past_Ia_rates[-1] = norm_rate


class SpinalCord(object):
    """ Spinal cord properties """

    def __init__(self, n_muscles, ext_muscles=None, flex_muscles=None):
        self.ext_muscles = ext_muscles
        self.flex_muscles = flex_muscles


def build_sc(sc_model_file, step_size):
    """ Spinal cord model """
    file = open(sc_model_file, "r")
    lines = file.readlines()
    n_muscles = len(lines) - 1
    sc_model = SpinalCord(n_muscles)
    ext_muscles = {}
    flex_muscles = {}
    for i in range(1, len(lines)):
        muscle = lines[i].split()
        if muscle[1] == 'ext':
            ext_muscles[muscle[0]] = Muscle(muscle[0], step_size, type=muscle[1], Ia_antagonists=muscle[2].split(','),
                                            Ia_delay=float(muscle[3]), Ia_w=float(muscle[4]),
                                            IaIn_w=list(map(float, muscle[5].split(','))), IaIn_IaIn_w=float(muscle[6]),
                                            MN_RC_w=float(muscle[7]), RC_MN_w=float(muscle[8]),
                                            RC_IaIn_w=float(muscle[9]), RC_RC_w=float(muscle[10]))
        elif muscle[1] == 'flex':
            flex_muscles[muscle[0]] = Muscle(muscle[0], step_size, type=muscle[1], Ia_antagonists=muscle[2].split(','),
                                             Ia_delay=float(muscle[3]), Ia_w=float(muscle[4]),
                                             IaIn_w=list(map(float, muscle[5].split(','))),
                                             IaIn_IaIn_w=float(muscle[6]),
                                             MN_RC_w=float(muscle[7]), RC_MN_w=float(muscle[8]),
                                             RC_IaIn_w=float(muscle[9]), RC_RC_w=float(muscle[10]))
    sc_model.ext_muscles = ext_muscles
    sc_model.flex_muscles = flex_muscles

    return sc_model