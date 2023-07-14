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
        self.ready = True

        time.sleep(2.0)

    def readLines(self):
        while True:
            lastLine = self.port.readline()
            print(f"received... {self.lastLine}")
            if lastLine == b'Cnc shield init!\r\n':
                self.ready = True
            if lastLine == b'MOVED\r\n':
                self.ready = True

    # y is forward x is sideways
    def goVector(self, vec, dist):
        if not self.isDone():
            raise "Not done"
        self.ready = False
        self.port.write(f"{vec[0]} {vec[1]} {dist} 0 0 ".encode())
        print("Sending... " + f"{vec[0]} {vec[1]} {dist} 0 0 ")
        self.port.flush()

    def goForward(self, dist):
        self.goVector([0, 1], dist)

    def rotateCW(self, deg):
        if not self.isDone():
            raise "Not done"
        self.ready = False
        self.port.write(f'0 0 0 0 {"{:.8f}".format(deg * math.pi / 180.0)}'.encode())
        print("Sending... " + f'0 0 0 0 {"{:.8f}".format(deg * math.pi / 180.0)}')
        self.port.flush()

    def isDone(self):
        return self.ready
