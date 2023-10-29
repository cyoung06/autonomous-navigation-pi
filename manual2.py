from peripherals.arduino_serial import MovingPlatform
import sys
import math

serial = MovingPlatform(sys.argv[1])

while True:
    a = input()
    dx, dy, w, dist = a.split(" ")
    serial.sendCommand(f'G {dx} {dy} {w} {dist}')
