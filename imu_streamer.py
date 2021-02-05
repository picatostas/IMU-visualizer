#!/usr/bin/python3
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import math
import serial
ser = serial.Serial(port='/dev/serial0', baudrate=115200)


def serial_print(msg):
    global ser
    assert(type(msg) == str)
    for i in msg:
        ser.write(bytes(i, 'utf-8'))


mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,  # In 0x68 Address
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)

mpu.configure()  # Apply the settings to the registers.

# Assuming gyro in home position
while True:
    start = time.time()

    ax, ay, az = tuple(mpu.readAccelerometerMaster())
    gx, gy, gz = tuple(mpu.readGyroscopeMaster())
    mx, my, mz = tuple(mpu.readMagnetometerMaster())
    serial_print("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n" %
                 (ax, ay, az, gx, gy, gz, mx, my, mz))
