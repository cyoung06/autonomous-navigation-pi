import random

import numpy


def calculateProbability(averageFunc, stdFunc, measurement):
    def actualStuff(loc):
        return numpy.prod(numpy.exp(-1/2 * (numpy.divide(measurement-averageFunc(loc),stdFunc(loc)) ** 2)))
        # proportional to probabilty of measuring smth in gaussian distribution.
    return actualStuff


def monteCarloLocalization(belief, updateFunc, probabilityFunc, size, gaussian):
    beliefs = [updateFunc(belief[i]) for i in range(len(belief))]
    weights = [probabilityFunc(belief[i]) for i in range(len(belief))]

    samples = random.choices(beliefs, weights=weights, k=size)
    samples = [gaussian(sample) for sample in samples]
    return samples
