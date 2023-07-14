import math
import threading
import time

import serial
from threading import Thread


class MovingPlatform:
    def __init__(self, port):
        self.port = serial.Serial(port)
        read = ""
        while b'Cnc shield init!\r\n' != read:
            read = self.port.readline()
            print(read)

        self.thread = threading.Thread(target=self.readLines)
        self.thread.start()
        self.lastLine = b"DONE"

        time.sleep(2.0)

    def readLines(self):
        while True:
            self.lastLine = self.port.readline()
            print(f"received... {self.lastLine}")

    # y is forward x is sideways
    def goVector(self, vec, dist):
        if not self.isDone():
            raise "Not done"
        self.lastLine = ""
        self.port.write(f"{vec[0]} {vec[1]} {dist} 0 0 ")
        print("Sending... " + f"{vec[0]} {vec[1]} {dist} 0 0 ")
        self.port.flush()

    def goForward(self, dist):
        self.goVector([0, 1], dist)

    def rotateCW(self, deg):
        if not self.isDone():
            raise "Not done"
        self.lastLine = ""
        self.port.write(f'0 0 0 0 {"{:.8f}".format(deg * math.pi / 180.0)}')
        print("Sending... " + f'0 0 0 0 {"{:.8f}".format(deg * math.pi / 180.0)}')
        self.port.flush()

    def isDone(self):
        return self.lastLine == b"MOVED\r\n"
