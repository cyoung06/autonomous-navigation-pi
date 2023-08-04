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
        self.thread.daemon = True
        self.thread.start()
        self.ready = True

        time.sleep(2.0)

    def readLines(self):
        while True:
            lastLine = self.port.readline()
            print(f"received... {lastLine}")
            if lastLine == b'Cnc shield init!\r\n':
                self.ready = True
            if lastLine == b'MOVED\r\n':
                self.ready = True

    # y is forward x is sideways

    def sendCommand(self, command):
        self.port.write(command.encode())
        print("Sending... " + command)
        self.port.flush()


    def goVector(self, vec, dist):
        if not self.isDone():
            raise "Not done"
        self.ready = False
        self.sendCommand(f"G {vec[0]} {vec[1]} 0 {dist / math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])}\n")

    def goForward(self, dist):
        self.goVector([0, 100], dist)

    def rotateCW(self, deg):
        if not self.isDone():
            raise "Not done"
        self.ready = False
        self.sendCommand(f'G 0 0 {"{:.8f}".format(deg * math.pi / 180.0)} 1\n')

    def stop(self):
        self.sendCommand(f"C\n")
    def resume(self):
        self.sendCommand(f"R\n")

    def cancel(self):
        self.sendCommand(f"S\n")
    def isDone(self):
        return self.ready
