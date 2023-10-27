import random

import numpy


def calculateProbability(averageFunc, stdFunc, measurement):
    def actualStuff(loc):
        vals = numpy.exp(-1/2 * (numpy.divide(measurement-averageFunc(loc), stdFunc(loc)) ** 2))
        print(vals)
        # proportional to probabilty of measuring smth in gaussian distribution.
    return actualStuff


def monteCarloLocalization(belief, updateFunc, probabilityFunc, size, gaussian):
    beliefs = [updateFunc(belief[i]) for i in range(len(belief))]
    print(f"After update: {beliefs}")
    weights = [probabilityFunc(belief[i]) for i in range(len(belief))]
    print(f"Weights: {weights}")
    samples = random.choices(beliefs, weights=weights, k=size)
    samples = [gaussian(sample) for sample in samples]
    return samples
