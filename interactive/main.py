from tkinter import *

root = Tk()
root.geometry('1000x1000')


import smbus2
from peripherals.wifi import get_nearby_routers
from peripherals.sensors_i2c import Sensors
from imusensor.MPU9250 import MPU9250

bus = smbus2.SMBus(1)
imu = MPU9250.MPU9250(bus, 0x68)
ultra = Sensors(bus, 0x11)


Label(root, text="lol", width= 500, height=10).pack()


def task():
    print(get_nearby_routers())

    root.after(2000, task)  # reschedule event in 2 seconds

root.after(2000, task)
root.mainloop()