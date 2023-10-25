import os.path
import sys
import threading
import pickle
import time
from os.path import dirname
import atexit

from picamera2 import Picamera2

import numpy
import numpy as np
from numpy import ndarray

from robot import Robot
from navigation.world import World, Cell, RelativePosition



sys.path.append(dirname(__file__))
print(sys.path)


def similarity(v1: ndarray, v2: ndarray):
    return numpy.linalg.norm(v1 - v2)


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
    atexit.register(exit_handler)

    print("Trying to connect to "+sys.argv[1])
    robot = Robot(sys.argv[1])
    if os.path.isfile('world.dat'):
        with open('world.dat', 'rb') as f:
            world = pickle.load(f)
    else:
        world = World(similarity)

    if sys.argv[2] == 'go':
        threading.Thread(target=runGUI).start()

    # now we navigate.

    # config = cam.create_still_configuration()
    # cam.configure(config)
    #
    # cam.start()


    lastCell = None

    time.sleep(10)
    grid = [[0] * 500 for i in range(0 ,500)]
    currPos = (250, 250)
    currDirection = 0 # y+, x+, y-, x-
    directions = [(0,1), (1,0), (0, -1), (-1, 0)]
    currDeg = robot.orientation[0]
    targetSensorDegrees = [currDeg, currDeg, currDeg, currDeg]
    robot.justRotate(90)
    targetSensorDegrees[1] = robot.orientation[0]
    robot.justRotate(90)
    targetSensorDegrees[2] = robot.orientation[0]
    robot.justRotate(90)
    targetSensorDegrees[3] = robot.orientation[0]
    robot.justRotate(90)
    print(f"Targets: {targetSensorDegrees}")


    currMoves = []
    visitedPoses = []


    while True:

        print(robot.orientation)
        pass

        def measurePosition() -> ndarray:
            measurements = 0
            currentMeasurement = robot.routers
            lastMeasurement = robot.routerUpdate

            currentRot = robot.orientation
            positionVector = [0] * (maxMacAddrs + 1)
            measurementsPer = [0] * (maxMacAddrs + 1)
            while measurements < 3:
                while robot.routerUpdate == lastMeasurement:
                    pass

                measurements += 1
                if len(macAddrMapping) < maxMacAddrs:
                    for (k, router) in currentMeasurement.items():
                        if k not in macAddrMapping:
                            macAddrMapping[k] = len(macAddrMapping)
                            macAddrsToListenTo.append(k)
                        if len(macAddrMapping) >= maxMacAddrs:
                            break
                if gui is not None:
                    gui.macAddrsToListenTo = macAddrsToListenTo

                for (k, v) in macAddrMapping.items():
                    if k in currentMeasurement:
                        positionVector[v] += currentMeasurement[k]
                        measurementsPer[v] += 1

            positionVector = [positionVector[i] / max(1, measurementsPer[i]) for i in
                              range(len(positionVector))]  # average them out
            positionVector[-1] = currentRot[1]
            positionVector = np.array(positionVector)
            return positionVector

        grid[currPos[1]][currPos[0]] = 1 # visited

        print(f"Visited:  {currPos}")

        robot.rotateTo(targetSensorDegrees[currDirection])
        print(f"Now at {robot.orientation} after coarse correction!")


        hasTarget = False
        for i in range(0,4):
            dir = directions[(currDirection + i) % 4]
            newPos = (currPos[0] + dir[0], currPos[1] + dir[1])

            print(f"Checking if i visited {newPos}? {grid[newPos[1]][newPos[0]]}")


            if (grid[newPos[1]][newPos[0]] != 0):
                continue

            if i != 0:
                print(f"B4 Rot: {robot.orientation}")

                currDirection = (currDirection + i) % 4
                robot.rotateTo(targetSensorDegrees[currDirection])
                print(f"B4 Rot: {robot.orientation}")
            print(f"Ultrasonic says {robot.ultrasonic}")
            if (robot.ultrasonic["forward"] != 0):  # fine to go
                grid[newPos[1]][newPos[0]] = -1 # can't go
                continue

            robot.goForward(500)

            visitedPoses.append(currPos)
            currPos = newPos
            hasTarget = True
            break
        if hasTarget:
            continue

        if len(visitedPoses) == 0:
            break

        prevPos = currPos
        currPos = visitedPoses.pop()
        wentDir = (prevPos[0] - currPos[0], prevPos[1] - currPos[1])
        realDir = directions.index(wentDir)
        currDirection = realDir
        robot.rotateTo(targetSensorDegrees[currDirection])

        robot.goForward(-100)

    # print(grid)
        # if not amIsafe():
        #     pass
        #
        # for i in range(0, 6):
        #     pos = measurePosition()
        #
        #     np_array = cam.capture_array()
        #     name = f"images/{int(time.time()*1000)}.jpg"
        #     cam.capture_file(name)
        #     with open(f'{name}.data', 'wb') as f:
        #         f.write(pickle.dumps([pos, robot.orientation]))
        #
        #     robot.platform.rotateCW(60)
        #     while amIsafe() and not robot.platform.isDone():
        #         pass
        #     if not amIsafe():
        #         robot.platform.stop()
        #
        #
        # if lastCell is not None:
        #     robot.platform.rotateCW(60)
        #     while amIsafe() and not robot.platform.isDone():
        #         pass
        #     if not amIsafe():
        #         robot.platform.stop()
        #
        #     robot.platform.goForward(1000)
        #     while amIsafe() and not robot.platform.isDone():
        #         pass
        #     if not amIsafe():
        #         robot.platform.stop()
        #
        # # listen for command
        #
        # positionVector = measurePosition()
        # dirVector = RelativePosition(-50 * 1.732, 50, 60)
        #
        # cell, probability = world.get_cell(positionVector)
        # if probability < 0.8:
        #     cell = Cell(positionVector)  # ehh
        #
        # if lastCell is not None:
        #     lastCell.connect(cell, dirVector)
        # world.add_cell(cell)
        # lastCell = cell
        #
        # print(world)
        # pass
