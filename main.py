import math
import os.path
import random
import sys
import threading
import pickle
import time
from os.path import dirname
import atexit
from statistics import mean
from typing import Tuple
from operator import sub, truediv

from navigation.localization import monteCarloLocalization, calculateProbability
from navigation.pathfinding import find_path

from picamera2 import Picamera2
import matplotlib.pyplot as plt

import numpy
import numpy as np
from numpy import ndarray

from robot import Robot
from navigation.world import World, Cell, RelativePosition

sys.path.append(dirname(__file__))
print(sys.path)


def similarity(v1: ndarray, v2: ndarray):
    dist = 0
    for i in range(0, len(v1)):
        if v1[i] != 0 and v2[i] != 0:
            dist += (v1[i] - v2[i]) ** 2
    return math.sqrt(dist)


if os.path.isfile('mac_addrs.dat'):
    with open('mac_addrs.dat', 'rb') as f:
        macAddrsToListenTo = pickle.load(f)
else:
    macAddrsToListenTo = []
macAddrMapping = {}
maxMacAddrs = 63

for addr in macAddrsToListenTo:
    macAddrMapping[addr] = len(macAddrMapping)

from interactive.statusgui import StatusGUI

gui: 'StatusGUI | None' = None


def runGUI():
    global gui
    gui = StatusGUI(robot, world)


if __name__ == '__main__':
    # cam = Picamera2()
    def exit_handler():
        print('My application is ending!')
        with open('world.dat', 'wb') as f:
            pickle.dump(world, f)
        with open('mac_addrs.dat', 'wb') as f:
            pickle.dump(macAddrsToListenTo, f)
        # cam.stop()

    print("Trying to connect to " + sys.argv[1])
    robot = Robot(sys.argv[1])
    if os.path.isfile('world.dat'):
        with open('world.dat', 'rb') as f:
            world = pickle.load(f)
    else:
        world = World(similarity)
    atexit.register(exit_handler)

    print("CELLS: ")
    for cell in world.nodes:
        print(f"Pos: {cell.position}")

    if sys.argv[2] == 'go':
        threading.Thread(target=runGUI).start()


    def measureSingle(lastMeasurement) -> Tuple[ndarray, int]:
        while robot.routerUpdate == lastMeasurement:
            pass
        measuredAt = robot.routerUpdate
        currentMeasurement = robot.routers
        positionVector = [math.inf] * (maxMacAddrs + 1)
        if len(macAddrMapping) < maxMacAddrs:
            for (k, router) in currentMeasurement.items():
                if k not in macAddrMapping:
                    macAddrMapping[k] = len(macAddrMapping)
                    macAddrsToListenTo.append(k)
                if len(macAddrMapping) >= maxMacAddrs:
                    break

        for (k, v) in macAddrMapping.items():
            if k in currentMeasurement:
                positionVector[v] = currentMeasurement[k]
        # positionVector[-1] = robot.orientation[0]
        return numpy.array(positionVector), measuredAt


    def measurePosition(samples) -> Tuple[ndarray, ndarray]:
        measurements = 0

        positionVectors = []
        sum = [0] * (maxMacAddrs + 1)
        measurementsPer = [0] * (maxMacAddrs + 1)
        lastMeasurement = robot.routerUpdate
        while measurements < samples:
            vec, lastMeasurement = measureSingle(lastMeasurement)
            positionVectors.append(vec)
            for i in range(len(vec)):
                if vec[i] != math.inf:
                    sum[i] += vec[i]
                    measurementsPer[i] += 1
            measurements += 1
        def smhmin(vec):
            if len(vec) == 0:
                return math.inf
            return mean(vec)
        meanVals = [math.inf if measurementsPer[i] == 0 else sum[i] /  measurementsPer[i] for i in range(len(sum))]
        devitation = [
            max(0.5, math.sqrt(smhmin([(positionVectors[j][i] - meanVals[i]) ** 2
                            for j in range(len(positionVectors))
                            if positionVectors[j][i] != math.inf])))
            for i in range(len(sum))
        ]
        return np.array(meanVals), np.array(devitation)


    pos = measurePosition(3)
    robot.justRotate(90)
    pos = measurePosition(3)
    robot.justRotate(-90)

    nodes = [[0] * 3 for i in range(0, 3)]
    wifi = [[None for x in range(0, 3)] for y in range(0, 3)]
    wifi = [[(numpy.array([        math.inf, 47.6       , 61.1       , 71.        , 64.9       ,
       70.6       , 72.3       , 58.3       , 60.2       ,         math.inf,
       82.1       , 77.8       , 95.        , 84.        , 69.3       ,
       77.3       , 76.3       , 90.        , 74.8       , 86.        ,
       76.4       , 76.        , 87.        , 81.66666667,         math.inf,
       89.1       , 87.        ,         math.inf, 92.        , 91.2       ,
               math.inf, 90.        , 88.        ,         math.inf,         math.inf,
               math.inf,         math.inf, 90.        , 89.7       , 83.        ,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf, 92.        ,         math.inf,         math.inf,         math.inf,
               math.inf, 95.        ,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf]), numpy.array([       math.inf, 6.34350061, 2.73678644, 3.        , 4.65725241,
       3.32264955, 1.41774469, 3.57910603, 2.35796522,        math.inf,
       2.7       , 3.6       , 0.5       , 0.5       , 3.1       ,
       1.9       , 3.63455637, 0.5       , 1.72046505, 0.5       ,
       1.42828569, 0.5       , 0.5       , 2.3570226 ,        math.inf,
       4.32319326, 5.72712843,        math.inf, 0.5       , 2.74954542,
              math.inf, 0.5       , 0.5       ,        math.inf,        math.inf,
              math.inf,        math.inf, 0.5       , 2.83019434, 2.52982213,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf, 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf, 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf])), (numpy.array([        math.inf, 43.7       , 54.6       , 60.        , 67.5       ,
       66.9       , 78.9       , 58.7       , 70.        ,         math.inf,
       78.2       , 76.3       , 80.5       , 84.        , 73.4       ,
       78.9       , 81.4       , 92.        , 83.        , 75.        ,
       79.4       , 76.        , 82.2       ,         math.inf,         math.inf,
       86.1       , 83.        ,         math.inf, 92.        , 89.        ,
               math.inf, 89.6       , 88.        ,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf, 86.        , 86.33333333,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf, 95.        ,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf]), numpy.array([       math.inf, 3.9       , 4.00499688, 3.34664011, 3.85356977,
       4.18210473, 3.08058436, 4.1484937 , 4.5607017 ,        math.inf,
       2.4       , 0.5       , 3.07408523, 0.5       , 0.8       ,
       2.02237484, 0.8       , 0.5       , 0.5       , 2.        ,
       1.9078784 , 0.5       , 2.4       ,        math.inf,        math.inf,
       1.37477271, 0.5       ,        math.inf, 0.5       , 0.5       ,
              math.inf, 0.8       , 0.5       ,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf, 2.19089023, 1.88561808,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf, 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf])), (numpy.array([ math.inf, 46.2, 57.4, 62.3, 69.7, 61.9, 75.1, 69.1, 67.5, 80. , 77. ,
       73.5, 78. , 80.8, 74.9, 81.6, 87. , 87.3, 79.8, 79.2, 79.5, 81. ,
       85. ,  math.inf,  math.inf, 86.6, 76. ,  math.inf, 85. ,  math.inf,  math.inf, 85.2,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf, 93. , 90. ,  math.inf,  math.inf,  math.inf,  math.inf,
        math.inf, 91. ,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf, 95. ,  math.inf,  math.inf,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf]), numpy.array([       math.inf, 3.91918359, 2.15406592, 5.3860932 , 4.9       ,
       4.80520551, 1.44568323, 4.94873721, 4.5       , 0.5       ,
       0.5       , 4.60977223, 0.5       , 3.91918359, 2.46779254,
       2.24499443, 0.5       , 2.1       , 0.5       , 1.83303028,
       2.53968502, 0.5       , 5.29150262,        math.inf,        math.inf,
       0.8       , 0.5       ,        math.inf, 0.5       ,        math.inf,
              math.inf, 1.98997487,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf, 0.5       , 0.5       ,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
       0.5       ,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf, 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf]))], [(numpy.array([        math.inf, 41.4       , 58.6       , 67.3       , 73.1       ,
       64.8       , 63.9       , 65.        , 62.5       , 82.        ,
       78.7       , 72.1       , 92.5       , 79.42857143, 69.8       ,
       78.4       , 74.8       , 89.9       , 74.3       , 76.6       ,
       78.1       ,         math.inf, 87.        , 85.        , 89.        ,
       89.        , 88.4       ,         math.inf, 87.        , 93.        ,
               math.inf, 90.8       , 85.        , 86.        ,         math.inf,
               math.inf, 95.        , 90.        , 86.        , 79.        ,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf, 92.        ,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf]), numpy.array([       math.inf, 3.13687743, 2.00997512, 2.19317122, 3.56230263,
       3.62767143, 1.86815417, 1.4832397 , 1.96214169, 0.5       ,
       2.64764046, 2.77308492, 2.5       , 0.72843136, 1.83303028,
       3.69323706, 1.32664992, 2.46779254, 2.00249844, 0.8       ,
       4.3       ,        math.inf, 0.5       , 0.5       , 0.5       ,
       0.5       , 0.91651514,        math.inf, 0.5       , 0.5       ,
              math.inf, 0.6       , 0.5       , 0.5       ,        math.inf,
              math.inf, 0.5       , 0.5       , 0.5       , 0.5       ,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf, 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf])), (numpy.array([ math.inf, 39.4, 56.1, 63.5, 68.8, 61.1, 74.6, 67.9, 68.5, 84.4, 77.8,
       71. ,  math.inf, 84. , 70.1, 84.5, 81.9, 90. , 80.7, 74.9, 77.4, 83. ,
       84. , 88. , 90.5, 90. , 83. ,  math.inf,  math.inf,  math.inf,  math.inf, 97. ,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf, 85. ,  math.inf,  math.inf,  math.inf,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf]), numpy.array([       math.inf, 1.9078784 , 3.7       , 4.5880279 , 3.34065862,
       2.91376046, 0.66332496, 1.92093727, 1.85741756, 1.2       ,
       1.46969385, 2.        ,        math.inf, 0.5       , 2.25610283,
       2.29128785, 3.3       , 0.5       , 1.41774469, 2.21133444,
       4.43170396, 0.5       , 0.5       , 0.5       , 1.5       ,
       0.5       , 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf, 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf, 0.5       ,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf])), (numpy.array([ math.inf, 38.1, 57. , 59.7, 62.5, 62.6, 73.6, 71. , 75.7, 80. , 78. ,
       75.3, 87. , 76.8, 79.5, 82.1, 86. , 85.2, 83. , 72.2, 80.4, 83.8,
       84. ,  math.inf, 90. , 87.8, 81. ,  math.inf, 82. , 91. ,  math.inf,  math.inf,  math.inf,
       89. ,  math.inf,  math.inf,  math.inf,  math.inf, 93. , 90. ,  math.inf,  math.inf,  math.inf,  math.inf,
        math.inf, 91. ,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf]), numpy.array([       math.inf, 3.93573373, 3.79473319, 2.83019434, 5.10392006,
       3.87814389, 2.93938769, 3.71483512, 2.9       , 0.5       ,
       2.64575131, 3.63455637, 0.5       , 1.2489996 , 2.29128785,
       0.5       , 0.5       , 1.2489996 , 3.31662479, 2.44131112,
       0.91651514, 3.42928564, 5.        ,        math.inf, 0.5       ,
       1.46969385, 0.5       ,        math.inf, 0.5       , 0.5       ,
              math.inf,        math.inf,        math.inf, 0.5       ,        math.inf,
              math.inf,        math.inf,        math.inf, 0.5       , 0.5       ,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
       0.5       ,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf]))], [(numpy.array([ math.inf, 41.4, 59.9, 61.1, 60.3, 66. , 74.2, 64.8, 62.8, 82.7, 77.5,
       73. , 90. , 87.4, 74.7, 82.8, 85. , 91. , 76.5, 78. , 83.5, 88. ,
        math.inf, 85. , 94. , 89. , 89. ,  math.inf, 87. ,  math.inf,  math.inf, 91. , 85. ,
       80. ,  math.inf,  math.inf, 95. ,  math.inf, 86. , 79. ,  math.inf, 88. ,  math.inf,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,
        math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf,  math.inf]), numpy.array([       math.inf, 1.8547237 , 2.62488095, 3.80657326, 4.36004587,
       2.96647939, 1.77763888, 2.22710575, 1.93907194, 0.5       ,
       1.74642492, 0.5       , 0.5       , 2.69072481, 0.5       ,
       0.87177979, 4.89897949, 0.5       , 1.80277564, 0.5       ,
       0.92195445, 0.5       ,        math.inf, 0.5       , 0.5       ,
       0.5       , 0.5       ,        math.inf, 0.5       ,        math.inf,
              math.inf, 0.5       , 0.5       , 0.5       ,        math.inf,
              math.inf, 0.5       ,        math.inf, 0.5       , 0.5       ,
              math.inf, 0.5       ,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf])), (numpy.array([        math.inf, 44.9       , 57.9       , 58.3       , 68.1       ,
       68.2       , 72.4       , 76.1       , 70.9       , 86.7       ,
       77.6       , 78.1       ,         math.inf, 84.        , 73.4       ,
       77.4       , 84.        , 90.6       , 86.7       , 73.        ,
       77.3       , 84.1       , 83.6       , 88.        , 90.6       ,
       90.        , 78.2       ,         math.inf,         math.inf, 94.        ,
               math.inf, 94.3       ,         math.inf, 88.16666667,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf, 85.        ,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf]), numpy.array([       math.inf, 4.32319326, 5.02891638, 1.61554944, 5.20480547,
       3.81575681, 2.15406592, 5.3       , 3.44818793, 0.9       ,
       2.87054002, 1.44568323,        math.inf, 0.5       , 3.03973683,
       2.15406592, 3.        , 1.2       , 0.9       , 0.5       ,
       1.67630546, 1.37477271, 1.28062485, 0.5       , 0.91651514,
       0.5       , 1.93907194,        math.inf,        math.inf, 0.5       ,
              math.inf, 0.9       ,        math.inf, 0.5       ,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf, 0.5       ,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf])), (numpy.array([        math.inf, 40.1       , 54.1       , 58.6       , 69.1       ,
       58.5       , 79.5       , 69.7       , 70.6       , 80.1       ,
       75.8       , 89.        ,         math.inf, 77.7       , 78.4       ,
       79.7       , 86.        , 87.8       , 85.5       , 70.9       ,
       79.        , 81.7       , 77.        ,         math.inf, 90.4       ,
       95.        , 80.1       ,         math.inf, 86.        , 91.        ,
               math.inf, 94.        ,         math.inf, 89.        ,         math.inf,
               math.inf,         math.inf,         math.inf, 90.14285714, 90.        ,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf,         math.inf,
               math.inf,         math.inf,         math.inf,         math.inf]), numpy.array([       math.inf, 2.16564078, 2.66270539, 2.90516781, 5.62938718,
       2.72946881, 2.76586334, 2.19317122, 1.62480768, 0.7       ,
       5.23067873, 0.5       ,        math.inf, 4.79687398, 1.74355958,
       2.60959767, 0.5       , 2.4       , 3.82753184, 1.13578167,
       2.0976177 , 5.55067563, 2.44948974,        math.inf, 0.8       ,
       0.5       , 0.5       ,        math.inf, 0.5       , 0.5       ,
              math.inf, 0.5       ,        math.inf, 0.5       ,        math.inf,
              math.inf,        math.inf,        math.inf, 1.80701581, 0.5       ,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf,        math.inf,
              math.inf,        math.inf,        math.inf,        math.inf]))]]
    currentLoc = (1, 1)
    currentDir = 0
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    stack = []
    # nodes[1][1] = 1
    # while True:
    #     x,y = currentLoc
    #     print(f"Visiting {x} {y}")
    #     refDir = currentDir
    #     found = False
    #     if wifi[y][x] is None:
    #         wifi[y][x] = measurePosition(10)
    # 
    #     for i in range(0, 4):
    #         dx, dy = directions[(i + refDir) % 4]
    #         if not (dy + y in range(0, 3) and dx + x in range(0, 3)):
    #             continue
    #         if nodes[dy+y][dx+x] == 0:
    #             robot.justRotate((i+refDir-currentDir) * 90)
    # 
    #             currentDir = (i + refDir) % 4
    #             # check
    #             blocked = False
    #             # robot.platform.rotateCW(500, 60)
    #             # while not robot.platform.isDone():
    #             #     if robot.ultrasonic["forward"] != 0 and robot.ultrasonic["forward"] < 30:
    #             #         print(f"smh blocked {robot.ultrasonic['forward']}")
    #             #         blocked = True
    #             # robot.justRotate(-30)
    # 
    #             if blocked:
    #                 nodes[dy + y][dx + x] = -1
    #                 print("BLOCKED NODE!")
    #                 continue
    # 
    #             nodes[dy+y][dx+x] = 1
    #             robot.goForward(1000)
    #             stack.append(currentLoc)
    #             currentLoc = (x+dx, y+dy)
    #             found = True
    #             break
    # 
    #     if found:
    #         continue
    # 
    #     if len(stack) == 0:
    #         break
    #     ox,oy = stack.pop()
    #     dx, dy = (ox - x, oy - y)
    #     targetDir = directions.index((dx,dy))
    #     robot.justRotate((targetDir - currentDir) * 90)
    #     robot.goForward(1000)
    #     currentDir = targetDir
    #     currentLoc = (ox, oy)
    # 
    # for y in range(0, 3):
    #     line = ''
    #     for x in range(0, 3):
    #         if nodes[y][x] == 1:
    #             line += " "
    #         elif nodes[y][x] == -1:
    #             line += 'X'
    #         elif nodes[y][x] == 0:
    #             line += '?'
    #     print(line)
    # print(wifi)
    # posVecMap = {}
    #
    # lines = [
    #     (pos, (0, 0), (0, 10000), 500)
    # ]
    #
    # currentBelief = (0, 0, 0)
    #
    # while len(lines) > 0:
    #     toPos, fromCoord, toCoord, segLen = lines.pop()
    #
    #     print(f"Measuring from {fromCoord} to {toCoord}, segLen {segLen}")
    #
    #     pathVec = tuple(map(sub, toCoord, fromCoord))
    #     pathLen = math.sqrt(pathVec[0] ** 2 + pathVec[1] ** 2)
    #     measurements = pathLen / segLen
    #
    #     angle = math.atan2(pathVec[0], pathVec[1])
    #     deltaPath = (pathVec[0] / measurements, pathVec[1] / measurements)
    #
    #     # move to angle
    #     # move to fromCoord
    #
    #     posVecMap[currentBelief] = measurePosition(3)
    #     print(posVecMap[currentBelief])
    #
    #     for i in range(math.ceil(measurements)):
    #         beliefX, beliefY, beliefTheta = currentBelief
    #         currentBelief = (beliefX + deltaPath[0], beliefY + deltaPath[1], beliefTheta)
    #         robot.goForward(segLen)
    #         posVecMap[currentBelief] = measurePosition(3)
    #         print(posVecMap[currentBelief])
    #
    # print(posVecMap)
    #
    # robot.goForward(-5000)
    # 300 by 300
    # arena is 100 by 100
    beliefs = [ (random.uniform(0, 3000), random.uniform(0, 3000)) for i in range(1000) ] # start with 1000 points
    print(f"Starting with: {beliefs}")
    currentDeg = currentDir * 90

    plt.ion()
    plt.show()

    plt.ylim(0, 3000)
    plt.xlim(0, 3000)

    old = plt.scatter([], [], alpha=0.1, c='#FF5555')
    new = plt.scatter([], [], alpha=0.9, c='#00FF00')

    arrow1 = plt.arrow(0,0,0,0)
    arrow2 = plt.arrow(0,0,0,0)
    while True:
        smh = random.randint(-5, 5) * 50
        if smh == 0:
            smh = 300

        rot = random.randint(-5, 5) * 30
        robot.justRotate(rot)
        currentDeg += rot
        robot.goForward(smh)
        def lolz(pos, access, idx):
            minY = math.floor(pos[1] / 1000)
            minX = math.floor(pos[0] / 1000)

            maxY = math.ceil(pos[1] / 1000)
            maxX = math.ceil(pos[0] / 1000)
            x, y = (pos[0]/1000, pos[1]/1000)
            if minY < 0 | minX < 0 | maxY >= 3 | maxX >= 3:
                return np.zeros(maxMacAddrs + 1)
            if access[minY][minX] is None or access[minY][maxX] is None or access[maxY][minX] is None or access[maxY][maxX] is None:
                return np.zeros(maxMacAddrs + 1)
            def dist(x1,y1,x2,y2):
                return math.sqrt((x1-x2)**2 + (y1-y2) ** 2)
            sumDist = dist(x,y, minX, minY) + dist(x,y, maxX, minY) + dist(x,y, minX, maxY) +  dist(x,y, maxX, maxY)
            val = access[minY][minX][idx] * dist(x,y, minX, minY)+\
                  access[minY][maxX][idx] * dist(x,y, maxX, minY)+\
                  access[maxY][minX][idx] * dist(x,y, minX, maxY)+\
                  access[maxY][maxX][idx] * dist(x,y, maxX, maxY)

            val /= sumDist
            return val

        old_beliefs = beliefs
        beliefs = monteCarloLocalization(
            beliefs,
            updateFunc=lambda t: (t[0]-smh*math.sin(rot * math.pi/180) + random.gauss(0, 500), t[1]+smh*math.cos(rot * math.pi/180) + random.gauss(0, 500)),
            probabilityFunc=calculateProbability(
                lambda pos: lolz(pos, wifi, 0)
                , lambda pos: lolz(pos, wifi, 1)
                , measureSingle(robot.routerUpdate)[0]
            ),
            size=980,
            gaussian=lambda t: (t[0] + random.gauss(0, 300), t[1] + random.gauss(0, 300))
        )


        def column(matrix, i):
            return [row[i] for row in matrix]


        old.set_data(column(old_beliefs, 0), column(old_beliefs, 1))
        new.set_data(column(beliefs, 0), column(beliefs, 1))
        arrow1.set_data(mean([x for x, y in old_beliefs]), mean([y for x, y in old_beliefs]),
                  -smh * math.sin(rot * math.pi / 180), smh * math.cos(rot * math.pi / 180))
        arrow2.set_data(mean([x for x, y in old_beliefs]), mean([y for x, y in old_beliefs]),
                  mean([x for x, y in beliefs]), mean([y for x, y in beliefs]))
        plt.draw()

        print(f'MEAN: {mean([x for x,y in beliefs])}, {mean([y for x,y in beliefs])}')
        beliefs += [ (random.uniform(0, 3000), random.uniform(0, 3000)) for i in range(20) ] # add 20 new points in case the robot has been kidnapped.
        input()
        time.sleep(2)
