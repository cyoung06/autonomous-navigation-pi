from peripherals.arduino_serial import MovingPlatform
import sys
import math

serial = MovingPlatform(sys.argv[1])

while True:
    a = input()
    deg, speed, dist = a.split(" ")
    deg = float(deg)
    speed = float(speed)
    dist = int(dist)

    if dist == 0:
        serial.rotateCW(speed, deg)
    else:
        dy = math.cos(deg* math.pi / 180.0) * speed
        dx = -math.sin(deg* math.pi / 180.0) * speed

        serial.goVector((dx,dy), dist)
