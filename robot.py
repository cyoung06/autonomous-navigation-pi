import numpy
import smbus2
from peripherals.wifi import get_nearby_routers
from peripherals.sensors_i2c import Sensors
from peripherals.arduino_serial import MovingPlatform
from peripherals.lcd import LiquidCrystal
from imusensor.MPU9250 import MPU9250
from peripherals.wifi import get_nearby_routers
from imusensor.filters import kalman

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
        self.imu.setGyroRange("GyroRangeSelect250DPS")
        self.imu.setAccelRange("AccelRangeSelect2G")
        self.imu.setLowPassFilterFrequency("AccelLowPassFilter184")
        self.sensors = Sensors(self.bus, 0x11)
        self.lcd = LiquidCrystal(self.bus)
        self.i2cUpdate = 0

        self.platform = MovingPlatform(port)

        self.imu.begin()

        self.imu.loadCalibDataFromFile("./calib.json")
        print("Caliberating Gyro...")
        self.imu.caliberateGyro()

        self.sensorfusion = kalman.Kalman()
        self.imu.readSensor()
        self.imu.computeOrientation()

        self.sensorfusion.roll = self.imu.roll
        self.sensorfusion.pitch = self.imu.pitch
        self.sensorfusion.yaw = self.imu.yaw



        threading.Thread(target=self._readI2C, daemon=True).start()
        threading.Thread(target=self._readRouters, daemon=True).start()



    def _readI2C(self):
        while True:
            self.imu.readSensor()
            self.imu.computeOrientation()
            self.old_orientation = [self.imu.yaw, self.imu.roll, self.imu.pitch]
            self.ultrasonic = self.sensors.readUltra()

            newTime = time.time()
            dt = newTime - self.i2cUpdate
            self.i2cUpdate = time.time()

            self.sensorfusion.computeAndUpdateRollPitchYaw(self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                                                      self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2], \
                                                      self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt)

            self.orientation = [self.sensorfusion.yaw, self.sensorfusion.roll, self.sensorfusion.pitch]

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

    def amIsafe(self):
            thresh = 7
            return self.ultrasonic["front"] < thresh and \
                self.ultrasonic["left"] < thresh and \
                self.ultrasonic["right"] < thresh



    def rotateTo(self, deg):
        supposedTobe = deg
        time.sleep(0.03)
        currentVal = self.orientation[0]
        toMove = currentVal - supposedTobe
        while toMove > 180:
            toMove -= 360
        while toMove < -180:
            toMove += 360
        while abs(toMove) > 1:
            print(f"Performing rotation!: {self.orientation} : {supposedTobe} : {toMove}")
            self.platform.rotateCW(numpy.clip([toMove], -10, 10)[0])

            welp = 0
            while True:
                while self.amIsafe() and not self.platform.isDone():
                    if welp > 0:
                        welp = 0
                        self.platform.resume()
                    pass
                if not self.amIsafe():
                    if welp == 0:
                        self.platform.stop()
                    welp += 1
                    time.sleep(0.1)
                    if welp == 10:
                        raise Exception("noooo")
                    continue
                break
            time.sleep(0.5)

            supposedTobe = deg
            currentVal = self.orientation[0]
            toMove = currentVal - supposedTobe
            while toMove > 180:
                toMove -= 360
            while toMove < -180:
                toMove += 360
    def justRotate(self, deg):
        self.platform.rotateCW(deg)
        welp = 0
        while True:
            while self.amIsafe() and not self.platform.isDone():
                if welp > 0:
                    welp = 0
                    self.platform.resume()
                pass
            if not self.amIsafe():
                if welp == 0:
                    self.platform.stop()
                welp += 1
                time.sleep(0.1)
                if welp == 10:
                    raise Exception("noooo")
                continue
            break
    def goForward(self, dist):
        self.platform.goForward(dist)
        welp = 0
        while True:
            while self.amIsafe() and not self.platform.isDone():
                if welp > 0:
                    welp = 0
                    self.platform.resume()
                pass
            if not self.amIsafe():
                if welp == 0:
                    self.platform.stop()
                welp += 1
                time.sleep(0.1)
                if welp == 10:
                    raise Exception("noooo")
                continue
            break
