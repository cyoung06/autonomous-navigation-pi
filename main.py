# MPU6050 9-DoF Example Printout
import sys
from os.path import dirname
from arduino_serial import MovingPlatform

sys.path.append(dirname(__file__))
print(sys.path)

from mpu9250_i2c import *

if __name__ == "__main__":
    platform = MovingPlatform(sys.argv[1])
    time.sleep(1)  # delay necessary to allow mpu9250 to settle


    print('recording data')
    while 1:
        if platform.isDone():
            platform.goForward(10)
        try:
            ax, ay, az, wx, wy, wz = mpu6050_conv()  # read and convert mpu6050 data
            mx, my, mz = AK8963_conv()  # read and convert AK8963 magnetometer data
        except:
            continue

        print('{}'.format('-' * 30))
        print('accel [g]: x = {0:2.2f}, y = {1:2.2f}, z {2:2.2f}= '.format(ax, ay, az))
        print('gyro [dps]:  x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(wx, wy, wz))
        print('mag [uT]:   x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(mx, my, mz))
        print('{}'.format('-' * 30))
        time.sleep(1)
