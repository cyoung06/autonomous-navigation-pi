# MPU6050 9-DoF Example Printout
import random
import sys
from os.path import dirname
from arduino_serial import MovingPlatform
from lcd import LiquidCrystal
import ip_utils

sys.path.append(dirname(__file__))
print(sys.path)

import time
import smbus2

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman
from sensors_i2c import Sensors
# kalman_filter = kalman.Kalman()

choices = [15, 30, 60, 45, -45, -15, -30, -60]

address = 0x68
bus = smbus2.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
ultra = Sensors(bus, 0x11)
lcd = LiquidCrystal(bus, 0x27)

lcd.lcd_display_string("IP ADDR", 1,0)
lcd.lcd_display_string(ip_utils.get_ip(), 2,0)

imu.begin()

imu.loadCalibDataFromFile("./calib.json")
imu.readSensor()
imu.computeOrientation()


currTime = time.time()
goal = {"end": True}
# kal_currTime = time.time()
if __name__ == "__main__":
    platform = MovingPlatform(sys.argv[1])
    time.sleep(1)  # delay necessary to allow mpu9250 to settle

    status = False
    with open("log.csv", "w") as log:
        # log.write("t,ax,ay,az,mx,my,mz,gx,gy,gz,roll,pitch,yaw\n")
        # print('recording data')
        while 1:
            if platform.isDone():
                if goal["end"]:
                    if status:
                        goal = {"type": "forward", "len": random.randint(2, 5) * 100, "curr": 0, "end": False}
                    else:
                        deg = choices[random.randint(0, len(choices)-1)]
                        goal = {"type": "turn", "deg": deg, "curr": 0, "end": False}
                    status = not status

                ultrasonic = ultra.readUltra()
                print(ultrasonic)
                welp = False
                for k, v in ultrasonic.items():
                    if v > 8 or v == 0:
                        welp = True
                if welp:
                    print("WELP!!!")
                else:
                    if goal["type"] == "forward":
                        length = 10 if goal["len"] > 0 else -10
                        platform.goForward(length)
                        goal["curr"] += length
                        if abs(goal["len"] - goal["curr"]) <= 10:
                            goal["end"] = True
                    else:
                        deg = 10 if goal["deg"] > 0 else -10
                        platform.rotateCW(deg)
                        goal["curr"] += deg
                        if abs(goal["deg"] - goal["curr"]) <= 10:
                            goal["end"] = True

            imu.readSensor()
            imu.computeOrientation()
            # time.sleep(5)
            # kal_newTime = time.time()
            # kal_dt = kal_newTime - kal_currTime
            # kal_currTime = kal_newTime

            # kalman_filter.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2],
            #                                            imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],
            #                                            imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], kal_dt)

            # print("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1],
            #                                                             imu.AccelVals[2]))
            # print("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
            # print("Mag x: {0} ; Mag y : {1} ; Mag z : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
            # print("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))

            # log.write('{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n'.format(
            #     time.time(),
            #     imu.AccelVals[0],
            #     imu.AccelVals[1],
            #     imu.AccelVals[2],
            #     imu.GyroVals[0],
            #     imu.GyroVals[1],
            #     imu.GyroVals[2],
            #     imu.MagVals[0],
            #     imu.MagVals[1],
            #     imu.MagVals[2],
            #     imu.roll,
            #     imu.pitch,
            #     imu.yaw))
            # time.sleep(0.001)
