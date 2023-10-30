import time

from peripherals.arduino_serial import MovingPlatform
import sys
import math
import numpy

serial = MovingPlatform(sys.argv[1])


needWait = False
def doStuff(dx, dy, w, dist):

    global needWait
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
        if i == 5 and not needWait:
            serial.ready = False
            needWait = True
        serial.sendCommand(f'M {vx[i]} {vy[i]} {-rad} {step}\n')
        if i > 5 or needWait:
            serial.waitForReady()
            serial.ready = False



while True:
    # a = input()
    doStuff(0, 500, 120, 0.25)
    doStuff(-250, 250 * math.sqrt(3), -240, 0.25)
    doStuff(250, 250 * math.sqrt(3), 120, 0.25)