import time

from peripherals.arduino_serial import MovingPlatform
import sys
import math
import numpy

serial = MovingPlatform(sys.argv[1])

while True:
    a = input()
    dx, dy, w, dist = a.split(" ")
    dx = float(dx)
    dy = float(dy)
    w = float(w)
    dist = float(dist)

    times = numpy.linspace(0, dist, int(dist * 10))

    # vx * cos theta
    theta = times * w * math.pi / 180
    vx = numpy.cos(-theta) * dx + numpy.sin(-theta) * dy
    vy = -numpy.sin(-theta) * dx + numpy.cos(-theta) * dy

    rad = w * math.pi / 180
    step = 0.01
    numpy.array([vx, vy])

    for i in range(len(vx)):
        serial.sendCommand(f'G {-vx[i]} {vy[i]} {-rad} {step}')
        time.sleep(0.1)
