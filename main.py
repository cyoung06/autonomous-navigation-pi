import math
import os.path
import sys
import threading
import pickle
import time
from os.path import dirname
import atexit
from navigation.pathfinding import find_path

from picamera2 import Picamera2

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
        if v1[i] != 0 and v2[i] !=0:
            dist += (v1[i]-v2[i]) ** 2
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
    atexit.register(exit_handler)

    print("Trying to connect to "+sys.argv[1])
    robot = Robot(sys.argv[1])
    if os.path.isfile('world.dat'):
        with open('world.dat', 'rb') as f:
            world = pickle.load(f)
    else:
        world = World(similarity)

    print("CELLS: ")
    for cell in world.nodes:
        print(f"Pos: {cell.position}")

    if sys.argv[2] == 'go':
        threading.Thread(target=runGUI).start()


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
            lastMeasurement = robot.routerUpdate

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

            print(positionVector)

        positionVector = [positionVector[i] / max(1, measurementsPer[i]) for i in
                          range(len(positionVector))]  # average them out
        positionVector[-1] = currentRot[0]
        positionVector = np.array(positionVector)
        return positionVector

    def getCell(pos):
        cell, prob = world.get_cell(pos)
        print(f"OMG MATCH! \nCell vec: {cell.position}\nCurr Vec: {pos}\ncosTheta {prob}")
        if prob > 10:
            cell = Cell(pos)
            world.add_cell(cell)
            if gui is not None:
                gui.newCell(cell)
        return cell, prob <= 10

    pos = measurePosition()
    robot.justRotate(90)
    robot.justRotate(-90)
    pos = measurePosition()
    lastCell, prob = world.get_cell(pos)
    if prob > 100 or prob < 0:
        lastCell = Cell(pos)
        world.add_cell(lastCell)

    toVisit = [ [lastCell, 0, 1000], [lastCell, 90, 1000], [lastCell, 180, 1000], [lastCell, -90, 1000]]
    while True:
        cell, dir, dist = toVisit.pop()
        print(f"Curr at {cell}")
        if lastCell != cell:
            path = find_path(lastCell.position, cell.position, world)
            print(f"Found path! {path}")
            for part in path:
                if part.rel_pos.rot != 0:
                    robot.justRotate(part.rel_pos.rot)
                else:
                    robot.goForward(part.rel_pos.dy)
                if gui is not None:
                    gui.focus(part.target)


            # navigate!

        robot.justRotate(dir)

        currCell, isNotNew = getCell(measurePosition())
        hmm = currCell.connect(lastCell, RelativePosition(0, 0, -dir))
        if gui is not None:
            gui.newConnection(hmm)
        lastCell = currCell
        if gui is not None:
            gui.focus(lastCell)

        if isNotNew:
            continue

        print(f"Ultra: {robot.ultrasonic}")
        if robot.ultrasonic["forward"] != 0:
            continue
        robot.goForward(dist)



        currCell, isNotNew = getCell(measurePosition())
        hmm = currCell.connect(lastCell, RelativePosition(0, -dist, 0))
        if gui is not None:
            gui.newConnection(hmm)
        lastCell = currCell
        if gui is not None:
            gui.focus(lastCell)
        if isNotNew:
            continue

        toVisit.append([currCell, 0, 1000])
        toVisit.append([currCell, 90, 1000])
        toVisit.append([currCell, 180, 1000])
        toVisit.append([currCell, -90, 1000])
        # move to cell


