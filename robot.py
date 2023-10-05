import smbus2
from peripherals.wifi import get_nearby_routers
from peripherals.sensors_i2c import Sensors
from peripherals.arduino_serial import MovingPlatform
from peripherals.lcd import LiquidCrystal
from imusensor.MPU9250 import MPU9250
from peripherals.wifi import get_nearby_routers
import ip_utils

class Robot:
    def __init__(self):
        self.orientation = [0,0,0]
        self.ultrasonic = {}
        self.routers = []

        self.bus = smbus2.SMBus(1)

        self.imu = MPU9250.MPU9250(self.bus, 0x68)
        self.sensors = Sensors(self.bus, 0x11)
        self.lcd = LiquidCrystal(self.bus)

        self.imu.begin()

        # self.imu.loadCalibDataFromFile("./calib.json")
        self.imu.readSensor()
        self.imu.computeOrientation()

        threading.Thread(target=self._readI2C, daemon=True).start()
        threading.Thread(target=self._readRouters, daemon=True).start()

    def _readI2C(self):
        self.orientation = [self.imu.yaw, self.imu.roll, self.imu.pitch]
        self.ultrasonic = self.sensors.readUltra()

    def _readRouters(self):
        self.routers = get_nearby_routers()