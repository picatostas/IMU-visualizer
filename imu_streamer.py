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
yaw = 0
pitch_gyro = 0
roll_gyro = 0
ts = 0.008

while True:
    start = time.time()

    Ax, Ay, Az = tuple(mpu.readAccelerometerMaster())
    Gx, Gy, Gz = tuple(mpu.readGyroscopeMaster())
    Mx, My, Mz = tuple(mpu.readMagnetometerMaster())
    pitch_acc = -1 * (180/math.pi * math.atan2(Ax, math.sqrt(Ay**2 + Az**2)))
    roll_acc = 180/math.pi * math.atan2(Ay, math.sqrt(Ax**2 + Az**2))
    # pitch_acc = -1 * (180/math.pi * math.atan2(Ax, Az))
    # roll_acc = 180/math.pi * math.atan2(Ay, Az)
    elapsed = time.time() - start
    pitch_gyro += Gy*ts
    roll_gyro += Gx*ts
    yaw += Gz*ts
    # complementary filter values
    pitch_comp = pitch_gyro*0.98 + pitch_acc*0.02
    roll_comp = roll_gyro*0.98 + roll_acc*0.02
    serial_print('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \r\n' % (
        roll_acc, pitch_acc, roll_gyro, pitch_gyro, roll_comp, pitch_comp, yaw))
    # this is for ensuring as constant ts as possible
    if elapsed < ts:
        time.sleep(ts - elapsed)
