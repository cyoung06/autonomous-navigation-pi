# MPU6050 9-DoF Example Printout
import random
import sys
from os.path import dirname
from arduino_serial import MovingPlatform

sys.path.append(dirname(__file__))
print(sys.path)

import time
import smbus

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman
kalman_filter = kalman.Kalman()

choices = [15, 30, 60, 45, -45, -15, -30, -60]

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

imu.loadCalibDataFromFile("./calib.json")

imu.readSensor()
imu.computeOrientation()
kalman_filter.roll = imu.roll
kalman_filter.pitch = imu.pitch
kalman_filter.yaw = imu.yaw


currTime = time.time()
kal_currTime = time.time()
if __name__ == "__main__":
    platform = MovingPlatform(sys.argv[1])
    time.sleep(1)  # delay necessary to allow mpu9250 to settle

    status = False
    with open("log.csv", "w") as log:
        log.write("t,ax,ay,az,mx,my,mz,gx,gy,gz,roll,pitch,yaw\n")
        print('recording data')
        while 1:
            # if platform.isDone():
            #     if status:
            #         platform.goForward(random.randint(2, 5) * 100)
            #     else:
            #         platform.rotateCW(choices[random.randint(0, len(choices)-1)])
            #     status = not status

            imu.readSensor()
            imu.computeOrientation()
            kal_newTime = time.time()
            kal_dt = kal_newTime - kal_currTime
            kal_currTime = kal_newTime

            kalman_filter.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2],
                                                       imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],
                                                       imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], kal_dt)

            print("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1],
                                                                        imu.AccelVals[2]))
            print("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
            print("Mag x: {0} ; Mag y : {1} ; Mag z : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
            print("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))

            log.write('{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n'.format(
                time.time(),
                imu.AccelVals[0],
                imu.AccelVals[1],
                imu.AccelVals[2],
                imu.GyroVals[0],
                imu.GyroVals[1],
                imu.GyroVals[2],
                imu.MagVals[0],
                imu.MagVals[1],
                imu.MagVals[2],
                kalman_filter.roll,
                kalman_filter.pitch,
                kalman_filter.yaw))
            time.sleep(0.001)
