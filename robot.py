import smbus2
from peripherals.wifi import get_nearby_routers
from peripherals.sensors_i2c import Sensors
from peripherals.arduino_serial import MovingPlatform
from peripherals.lcd import LiquidCrystal
from imusensor.MPU9250 import MPU9250
from peripherals.wifi import get_nearby_routers
import ip_utils
import time
import threading

class Robot:
    def __init__(self, port):
        self.orientation = [0,0,0]
        self.ultrasonic = {}
        self.routers = {}
        self.routerUpdate = 0

        self.bus = smbus2.SMBus(1)

        self.imu = MPU9250.MPU9250(self.bus, 0x68)
        self.sensors = Sensors(self.bus, 0x11)
        self.lcd = LiquidCrystal(self.bus)
        self.i2cUpdate = 0

        self.platform = MovingPlatform(port)

        self.imu.begin()

        self.imu.loadCalibDataFromFile("./calib.json")
        self.imu.readSensor()
        self.imu.computeOrientation()

        threading.Thread(target=self._readI2C, daemon=True).start()
        threading.Thread(target=self._readRouters, daemon=True).start()



    def _readI2C(self):
        while True:
            self.imu.readSensor()
            self.imu.computeOrientation()
            self.orientation = [self.imu.yaw, self.imu.roll, self.imu.pitch]
            self.ultrasonic = self.sensors.readUltra()
            self.i2cUpdate = time.time()
            time.sleep(0.01)

    def _readRouters(self):
        while True:
            routers = get_nearby_routers()
            dict = {}
            for router in routers:
                dict[router.bssid] = router.rssi

            self.routers = dict

            self.routerUpdate = time.time()
            time.sleep(0.01)
