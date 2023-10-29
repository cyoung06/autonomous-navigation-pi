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
        serial.rotateCW(deg)
    else:
        dx = math.cos(deg* math.pi / 180.0) * speed
        dy = -math.sin(deg* math.pi / 180.0) * speed

        serial.goVector((dx,dy), dist)
