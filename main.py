import sys
import threading
from os.path import dirname

import numpy
import numpy as np
from numpy import ndarray

from robot import Robot
from navigation.world import World, Cell

sys.path.append(dirname(__file__))
print(sys.path)


def similarity(v1: ndarray, v2: ndarray):
    return numpy.linalg.norm(v1 - v2)


macAddrsToListenTo = []
macAddrMapping = {}
maxMacAddrs = 63

for addr in macAddrsToListenTo:
    macAddrMapping[addr] = len(macAddrMapping)

from interactive.statusgui import StatusGUI

gui: StatusGUI | None = None

def runGUI():
    global gui
    gui = StatusGUI(robot, world)


if __name__ == '__main__':
    robot = Robot()
    world = World(similarity)

    threading.Thread(target=runGUI).start()

    # now we navigate.

    while True:
        # listen for command
        currentMeasurement = robot.routers
        currentRot = robot.orientation

        if len(macAddrMapping) < maxMacAddrs:
            for (k, router) in currentMeasurement.items():
                if k not in macAddrMapping:
                    macAddrMapping[k] = len(macAddrMapping)
                    macAddrsToListenTo.append(k)
                if len(macAddrMapping) >= maxMacAddrs:
                    break
        if gui is not None:
            gui.macAddrsToListenTo = macAddrsToListenTo

        positionVector = [0] * (maxMacAddrs + 1)
        for (k, v) in macAddrMapping.items():
            positionVector[v] = currentMeasurement[k]
        positionVector[-1] = currentRot[1]

        world.add_cell(Cell(np.array(positionVector)))

        pass
