import math
import threading
import time

import serial
from threading import Thread


class MovingPlatform:
    def __init__(self, port):
        self.port = serial.Serial(port, baudrate=115200)
        read = ""
        while b'Cnc shield init!\r\n' != read:
            read = self.port.readline()
            print(read)

        self.thread = threading.Thread(target=self.readLines)
        self.thread.daemon = True
        self.thread.start()
        self.ready = True
        self.received = False

    def readLines(self):
        while True:
            lastLine = self.port.readline()
            print(f"received... {lastLine}")
            if lastLine == b'Cnc shield init!\r\n':
                self.ready = True
            if lastLine == b'MOVED\r\n':
                self.ready = True
            if lastLine == b'RECEIVED\r\n':
                self.received = True

    # y is forward x is sideways

    def sendCommand(self, command):
        self.received = False
        print("Sending... " + command)
        self.port.write(command.encode())
        self.port.flush()
        while not self.received:
            pass


    def goVector(self, vec, dist):
        if not self.isDone():
            raise "Not done"
        self.ready = False
        self.sendCommand(f"G {vec[0]} {vec[1]} 0 {dist / math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])}\n")

    def goForward(self, dist):
        self.goVector([0, 10000], dist)

    def rotateCW(self, speed, degs):
        if not self.isDone():
            raise "Not done"
        self.ready = False
        self.sendCommand(f'G 0 0 {"{:.8f}".format(speed * math.pi / 180.0)} {degs/speed}\n')

    def waitForReady(self):
        while not self.isDone():
            pass
        return

    def stop(self):
        self.sendCommand(f"C\n")
    def resume(self):
        self.sendCommand(f"R\n")

    def cancel(self):
        self.sendCommand(f"S\n")
    def isDone(self):
        return self.ready
