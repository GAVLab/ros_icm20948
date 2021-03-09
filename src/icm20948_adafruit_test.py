# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import busio
import os
import numpy as np
from adafruit_icm20x import ICM20948,AccelRange,GyroRange
from ahrs import filters
from scipy.spatial.transform import Rotation as R

i2c = busio.I2C(board.SCL, board.SDA)
icm = ICM20948(i2c)
icm.accelerometer_range = AccelRange.RANGE_4G # Options: RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G
time.sleep(0.1)
print("Acceleration Range: %dG" % ((icm.accelerometer_range+1)*2))
if icm.gyro_range == 0:
    gyro_range = 250
elif icm.gyro_range == 1:
    gyro_range = 500
elif icm.gyro_range == 2:
    gyro_range = 1000
elif icm.gyro_range == 3:
    gyro_range = 2000
else:
    gyro_range = i
print("Gyro Range: %d degrees/s" % (gyro_range))
time.sleep(1)

frequency = 100  # frequency in Hertz

madgwick = filters.Madgwick()
Q = np.array([1.0, 0.0, 0.0, 0.0])


while True:
    acc_data = icm.acceleration # linear acceleration (m/s^2) x,y,z
    gyr_data = icm.gyro # angular velocity (rad/s) x,y,z
    mag_data = tuple(i for i in icm.magnetic) # magnetic field (uT) x,y,z
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (acc_data))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (gyr_data))
    print("Magnetometer X:%.2f, Y: %.2f, Z: %.2f uT" % (mag_data))
    madgwick.Dt = 1/frequency
    Q = madgwick.updateMARG(Q,acc=np.array(acc_data),gyr=np.array(gyr_data),mag=np.array(mag_data))
    print("Quaternion Orientation: "+str(Q))
    r = R.from_quat(Q)
    print("Euler Orientation: "+str(r.as_euler('zyx',degrees=True))) 
    print("")
    time.sleep(1/frequency)

