import numpy as np


class Muscle():
    """ Muscle properties """
    def __init__(self, name, step_size, Ia_antagonists=[], Ia_delay=30, Ia_weight=1, recip_weights=[]):
        self.name = name
        self.Ia_delay = Ia_delay
        self.Ia_antagonists = Ia_antagonists
        self.past_Ia_rates = np.zeros(max(1, int(self.Ia_delay/(step_size*1000))))
        self.past_fiber_l = 0
        self.Ia_weight = Ia_weight
        if len(recip_weights) == 1:
            self.recip_weights = recip_weights[0]*np.ones(len(Ia_antagonists))
        else:
            self.recip_weights = recip_weights

    def update_past_Ia_rate(self, Ia_rate, delay):
        """ Update Ia rates """
        self.past_Ia_rates[delay] = Ia_rate


class SpinalCord():
    """ Spinal circuit properties """
    def __init__(self, n_muscles):
        self.n_muscles = n_muscles
        self.muscles = {}
        self.Ia_w = np.ones(n_muscles)
        self.recip_matrix = np.zeros((n_muscles, n_muscles))
        self.recip_w = np.zeros((n_muscles, n_muscles))
        self.mn_rates = np.zeros(n_muscles)
        self.delayed_Ia_rates = np.zeros(n_muscles)
        self.Ia_In_rates = np.zeros(n_muscles)

    def SC_model(self, sc_model_file, muscle_names, step_size):
        """ Spinal circuit properties from spinal cord model file """
        file = open(sc_model_file, "r")
        lines = file.readlines()
        for i in range(1, len(lines)):
            muscle = lines[i].split()
            self.muscles[muscle[0]] = Muscle(muscle[0], step_size, Ia_antagonists=muscle[2].split(','),
                                         Ia_delay=int(muscle[3]), Ia_weight=float(muscle[4]),
                                         recip_weights=list(map(float, muscle[5].split(','))))
        for muscle in muscle_names:
            if muscle not in self.muscles:
                self.muscles[muscle] = Muscle(muscle, step_size)

        for i in range(len(muscle_names)):
            self.Ia_w[i] = self.muscles[muscle_names[i]].Ia_weight
            for j in range(len(self.muscles[muscle_names[i]].Ia_antagonists)):
                muscle = self.muscles[muscle_names[i]].Ia_antagonists[j]
                self.recip_matrix[i, muscle_names.index(muscle)] = 1
                self.recip_w[i, muscle_names.index(muscle)] = self.muscles[muscle_names[i]].recip_weights[j]

    def update_SC_rates(self, j, control, model, muscle_names, step_size):
        """ Update spinal circuit rates """
        for i in range(self.n_muscles):
            muscle = self.muscles[muscle_names[i]]
            self.delayed_Ia_rates[i] = muscle.past_Ia_rates[(j - 1) % (max(1, int(muscle.Ia_delay / (step_size * 1000))))]
            self.Ia_In_rates[i] = synapse([self.delayed_Ia_rates[i]], [1])
        in_inputs = np.multiply(self.recip_matrix, self.Ia_In_rates)
        for i in range(self.n_muscles):
            self.mn_rates[i] = synapse(np.array([self.delayed_Ia_rates[i], control[i]]),
                                  np.array([self.Ia_w[i], 1]), in_inputs[i], self.recip_w[i])
            muscle = model.model.getMuscles().get(i)
            self.muscles[muscle_names[i]].update_past_Ia_rate(prochazka_Ia_rate(
                model, muscle, self.muscles[muscle_names[i]]),
                (j - 1) % (max(1, int(self.muscles[muscle_names[i]].Ia_delay / (step_size * 1000)))))


def prochazka_Ia_rate(model, muscle, muscle_model, a=4.3, b=2, c=10):
    """ Compute Prochazka Ia rate """
    past_fiber_l = muscle_model.past_fiber_l
    fiber_l = muscle.getFiberLength(model.state) * 1000
    muscle_model.past_fiber_l = fiber_l
    if past_fiber_l == 0:
        fiber_v = 0.01
    else:
        fiber_v = (fiber_l - past_fiber_l)/model.step_size
    opt_l = muscle.getOptimalFiberLength() * 1000
    max_v = muscle.getMaxContractionVelocity() * opt_l
    rate = a * np.sign(fiber_v) * np.exp(0.6 * np.log(max(min(abs(fiber_v), max_v), 0.01))) + b * (
                min(fiber_l, 1.5 * opt_l) - opt_l) + c
    return max(rate / (a * np.exp(0.6 * np.log(max_v)) + b * 0.5 * opt_l + c), 0)


def sigmoid(x, alpha=1, beta=0.5):
    """ Sigmoid function """
    return 1/(1+np.exp(-alpha*(x-beta)))


def synapse(exc_inputs, exc_weights, in_inputs=None, in_weights=None, w0=0, alpha=6, beta=0.5):
    """ Integrate inputs at a synapse """
    if in_inputs is None:
        return sigmoid(np.sum(np.multiply(exc_inputs, exc_weights)) + w0, alpha, beta) - sigmoid(0, alpha, beta)
    else:
        return sigmoid(max(np.sum(np.multiply(exc_inputs, exc_weights)) -
                           np.sum(np.multiply(in_inputs, in_weights)) + w0, 0),
                       alpha, beta) - sigmoid(0, alpha, beta)


def muscle_excit(mn_rate, w1=1, w0=0, alpha=6, beta=0.5):
    """ Compute muscle excitation from motorneuron rate """
    return sigmoid(w1*mn_rate + w0, alpha, beta) - sigmoid(0, alpha, beta)
