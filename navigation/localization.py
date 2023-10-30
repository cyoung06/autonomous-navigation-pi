import random

import numpy
import numpy as np


def calculateProbability(averageFunc, stdFunc, measurement):
    def actualStuff(loc):
        avg = averageFunc(loc)
        std = stdFunc(loc)
        invalidIdx = numpy.in1d(measurement, [numpy.nan, numpy.inf], invert=True) & numpy.in1d(avg, [numpy.nan, numpy.inf], invert=True) & numpy.in1d(std, [numpy.nan, numpy.inf], invert=True)

        print(f'to Expect: {avg[invalidIdx]}')
        print(f'found: {measurement[invalidIdx]}')
        vals = 1 / (std[invalidIdx] * numpy.pi * 2) * numpy.exp(-1/2 * (numpy.divide(measurement[invalidIdx] - avg[invalidIdx], std[invalidIdx]) ** 2))
        print(vals)
        return numpy.prod(vals)
        # proportional to probabilty of measuring smth in gaussian distribution.
    return actualStuff


def monteCarloLocalization(belief, updateFunc, probabilityFunc, size, gaussian):
    beliefs = np.array([updateFunc(belief[i]) for i in range(len(belief))])
    print(f"After update: {beliefs}")
    weights = np.array([probabilityFunc(belief[i]) for i in range(len(belief))])
    filter = numpy.in1d(weights, [numpy.nan], invert=True)
    print(f"Weights: {weights[filter]}")

    samples = random.choices(beliefs[filter], weights=weights[filter], k=size)
    samples = [gaussian(sample) for sample in samples]
    return samples
