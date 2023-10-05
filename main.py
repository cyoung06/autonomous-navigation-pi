import sys
import threading
from os.path import dirname

from robot import Robot
from navigation.world import World


sys.path.append(dirname(__file__))
print(sys.path)


def similarity(v1, v2):
    return v1-v2

macAddrMapping = {

}

from interactive.statusgui import StatusGUI

def runGUI():
    gui = StatusGUI(robot, world)


if __name__ == '__main__':
    robot = Robot()
    world = World(similarity)

    threading.Thread(target=runGUI).start()



