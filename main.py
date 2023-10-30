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

    nodes = [[0] * 100 for i in range(0, 100)]
    wifi = [[None for x in range(0, 100)] for y in range(0, 100)]
    currentLoc = (50, 50)
    currentDir = 0
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    stack = []
    nodes[50][50] = 1
    while True:
        x,y = currentLoc
        print(f"Visiting {x} {y}")
        refDir = currentDir
        found = False
        if wifi[y][x] is None:
            wifi[y][x] = measurePosition(10)

        for i in range(0, 4):
            dx, dy = directions[(i + refDir) % 4]
            if not (dy + y in range(0, 100) and dx + x in range(0, 100)):
                continue
            if nodes[dy+y][dx+x] == 0:
                robot.justRotate((i+refDir-currentDir) * 90 - 30)

                currentDir = (i + refDir) % 4
                # check
                blocked = False
                robot.platform.rotateCW(500, 60)
                while not robot.platform.isDone():
                    if robot.ultrasonic["forward"] != 0 and robot.ultrasonic["forward"] < 30:
                        print(f"smh blocked {robot.ultrasonic['forward']}")
                        blocked = True
                robot.justRotate(-30)

                if blocked:
                    nodes[dy + y][dx + x] = -1
                    print("BLOCKED NODE!")
                    continue

                nodes[dy+y][dx+x] = 1
                robot.goForward(500)
                stack.append(currentLoc)
                currentLoc = (x+dx, y+dy)
                found = True
                break

        if found:
            continue

        if len(stack) == 0:
            break
        ox,oy = stack.pop()
        dx, dy = (ox - x, oy - y)
        targetDir = directions.index((dx,dy))
        robot.justRotate((targetDir - currentDir) * 90)
        robot.goForward(500)
        currentDir = targetDir
        currentLoc = (ox, oy)

    for y in range(0, 100):
        line = ''
        for x in range(0, 100):
            if nodes[y][x] == 1:
                line += " "
            elif nodes[y][x] == -1:
                line += 'X'
            elif nodes[y][x] == 0:
                line += '?'
        print(line)
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
    beliefs = [ (random.uniform(0, 50000), random.uniform(0, 50000)) for i in range(1000) ] # start with 1000 points
    print(f"Starting with: {beliefs}")
    currentDeg = currentDir * 90

    plt.ion()
    plt.show()

    plt.ylim(0, 50000)
    plt.xlim(0, 50000)

    old = plt.scatter([], [], alpha=0.1, c='#FF5555')
    new = plt.scatter([], [], alpha=0.9, c='#00FF00')

    arrow1 = plt.arrow(0,0,0,0)
    arrow2 = plt.arrow(0,0,0,0)
    while True:
        smh = random.randint(-5, 5) * 250
        if smh == 0:
            smh = 1500

        rot = random.randint(-5, 5) * 30
        robot.justRotate(rot)
        currentDeg += rot
        robot.goForward(smh)
        def lolz(pos, access, idx):
            minY = math.floor(pos[1] / 500)
            minX = math.floor(pos[0] / 500)

            maxY = math.ceil(pos[1] / 500)
            maxX = math.ceil(pos[0] / 500)
            x, y = (pos[0]/500, pos[1]/500)
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
        beliefs += [ (random.uniform(0, 50000), random.uniform(0, 50000)) for i in range(20) ] # add 20 new points in case the robot has been kidnapped.
        input()
        time.sleep(2)
