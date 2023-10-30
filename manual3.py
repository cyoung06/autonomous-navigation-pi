import time

from peripherals.arduino_serial import MovingPlatform
import sys
import math
import numpy

serial = MovingPlatform(sys.argv[1])


cnt = 0
def doStuff(dx, dy, w, dist):

    global cnt
    times = numpy.linspace(0, dist, int(dist * 100))

    # vx * cos theta
    theta = times * w * math.pi / 180
    vx = numpy.cos(-theta) * dx + numpy.sin(-theta) * dy
    vy = -numpy.sin(-theta) * dx + numpy.cos(-theta) * dy

    rad = w * math.pi / 180
    step = 0.01
    numpy.array([vx, vy])

    for i in range(len(vx)):
        if cnt == 5:
            serial.ready = False
        cnt += 1
        serial.sendCommand(f'M {vx[i]} {vy[i]} {-rad} {step}\n')
        if cnt > 5:
            serial.waitForReady()
            serial.ready = False



while True:
    # a = input()
    doStuff(0, 500, 120, 0.25)
    input()
    # doStuff(-250, 250 * math.sqrt(3), -240, 0.25)
    # doStuff(250, 250 * math.sqrt(3), 120, 0.25)