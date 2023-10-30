import time

from peripherals.arduino_serial import MovingPlatform
import sys
import math
import numpy

serial = MovingPlatform(sys.argv[1])

def doStuff(dx, dy, w, dist):

    times = numpy.linspace(0, dist, int(dist * 10))

    # vx * cos theta
    theta = times * w * math.pi / 180
    vx = numpy.cos(-theta) * dx + numpy.sin(-theta) * dy
    vy = -numpy.sin(-theta) * dx + numpy.cos(-theta) * dy

    rad = w * math.pi / 180
    step = 0.1
    numpy.array([vx, vy])

    cnt = 0
    for i in range(len(vx)):
        if i > 5:
            serial.ready = False
        serial.sendCommand(f'M {vx[i]} {vy[i]} {-rad} {step}\n')
        if i > 5:
            serial.waitForReady()



while True:
    # a = input()
    doStuff(0, 500, 30, 1)
    doStuff(250, 250 * math.sqrt(3), -60, 1)
    doStuff(-250, 250 * math.sqrt(3), 30, 1)