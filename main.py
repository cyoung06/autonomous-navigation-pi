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
    cam = Picamera2()
    def exit_handler():
        print('My application is ending!')
        with open('world.dat', 'wb') as f:
            pickle.dump(world, f)
        with open('mac_addrs.dat', 'wb') as f:
            pickle.dump(macAddrsToListenTo, f)
        cam.stop()


    atexit.register(exit_handler)

    robot = Robot(sys.argv[1])
    if os.path.isfile('world.dat'):
        with open('world.dat', 'rb') as f:
            world = pickle.load(f)
    else:
        world = World(similarity)

    threading.Thread(target=runGUI).start()

    # now we navigate.

    config = cam.create_still_configuration()
    cam.configure(config)

    cam.start()


    lastCell = None
    while True:
        def amIsafe():
            thresh = 10
            return robot.ultrasonic["front"] < thresh and \
                robot.ultrasonic["left"] < thresh and \
                robot.ultrasonic["right"] < thresh

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
                    positionVector[v] += currentMeasurement[k]
                    measurementsPer[v] += 1

            positionVector = [positionVector[i] / max(1, measurementsPer[i]) for i in
                              range(len(positionVector))]  # average them out
            positionVector[-1] = currentRot[1]
            positionVector = np.array(positionVector)
            return positionVector

        if not amIsafe():
            pass

        for i in range(0, 6):
            pos = measurePosition()

            np_array = cam.capture_array()
            name = f"{int(time.time()*1000)}.jpg"
            cam.capture_file(name)
            with open(f'{name}.data', 'wb') as f:
                f.write(pickle.dumps([pos, robot.orientation]))

            robot.platform.rotateCW(60)


        if lastCell is not None:
            robot.platform.rotateCW(60)
            robot.platform.goForward(100)


        # listen for command

        positionVector = measurePosition()
        dirVector = RelativePosition(-50 * 1.732, 50, 60)

        cell, probability = world.get_cell(positionVector)
        if probability < 0.8:
            cell = Cell(positionVector)  # ehh

        if lastCell is not None:
            lastCell.connect(cell, dirVector)
        world.add_cell(cell)
        lastCell = cell

        print(world)
        pass
