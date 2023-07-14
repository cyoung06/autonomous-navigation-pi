# MPU6050 9-DoF Example Printout
import random
import sys
from os.path import dirname
from arduino_serial import MovingPlatform

sys.path.append(dirname(__file__))
print(sys.path)

from mpu9250_i2c import *

choices = [15, 30, 60, 90, -15, -30, -60, -90, -180, 180]

if __name__ == "__main__":
    platform = MovingPlatform(sys.argv[1])
    time.sleep(1)  # delay necessary to allow mpu9250 to settle

    status = False
    with open("log.csv", "a") as log:
        print('recording data')
        while 1:
            if platform.isDone():
                if status:
                    platform.goForward(random.randint(5, 10) * 10)
                else:
                    platform.rotateCW(choices[random.randint(0, len(choices)-1)])
                status = not status
            try:
                ax, ay, az, wx, wy, wz = mpu6050_conv()  # read and convert mpu6050 data
                mx, my, mz = AK8963_conv()  # read and convert AK8963 magnetometer data
            except:
                continue

            log.write('{0},{1},{2},{3},{4},{5},{6},{7},{8}\n'.format(ax,ay,az,wx,wy,wz,mx,my,mz))
            time.sleep(0.03)
