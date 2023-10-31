import random

import numpy
import numpy as np


def calculateProbability(averageFunc, stdFunc, measurement):
    def actualStuff(loc):
        avg = averageFunc(loc)
        std = stdFunc(loc)
        invalidIdx = numpy.isfinite(avg) & numpy.isfinite(std) & numpy.isfinite(measurement)

        # print(f'to Expect: {avg[invalidIdx]}')
        # print(f'found: {measurement[invalidIdx]}')
        vals = 1 / (std[invalidIdx] * numpy.pi * 2) * numpy.exp(-1/2 * (numpy.divide(measurement[invalidIdx] - avg[invalidIdx], std[invalidIdx]) ** 2))
        if len(vals) == 0:
            return numpy.array(numpy.nan)
        return numpy.prod(vals)
        # proportional to probabilty of measuring smth in gaussian distribution.
    return actualStuff


def monteCarloLocalization(belief, updateFunc, probabilityFunc, size, gaussian):
    beliefs = np.array([updateFunc(belief[i]) for i in range(len(belief))])
    print(f"After update: {beliefs}")
    weights = np.array([probabilityFunc(belief[i]) for i in range(len(belief))])
    filter = numpy.isfinite(weights)
    print(f"Weights: {weights[filter]}")

    samples = random.choices(beliefs[filter], weights=weights[filter], k=size)
    samples = [gaussian(sample) for sample in samples]
    return samples
