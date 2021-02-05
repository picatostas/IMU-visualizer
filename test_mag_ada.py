#!/usr/bin/python3

from smbus2 import SMBus, i2c_msg
import math
import time
import serial

ser = serial.Serial(port='/dev/serial0', baudrate=115200)
def serial_print(msg):
    global ser
    assert(type(msg) == str)
    for i in msg:
        ser.write(bytes(i, 'utf-8'))


i2c_bus = SMBus(1)


# FXOS8700CQ Mag + Accel
FXOS8700CQ_I2C_ADDR = 0x1f
FXOS8700CQ_CTRL_REG_1 = 0x2A
FXOS8700CQ_CTRL_REG_2 = 0x2B
FXOS8700CQ_CTRL_REG_3 = 0x2C
FXOS8700CQ_CTRL_REG_4 = 0x2C
FXOS8700CQ_CTRL_REG_5 = 0x2E
FXOS8700CQ_M_CTRL_REG_1 = 0x5B
FXOS8700CQ_M_CTRL_REG_2 = 0x5C
FXOS8700CQ_M_CTRL_REG_3 = 0x5D
FXOS8700CQ_ACCEL_XOUT_H = 0x01
FXOS8700CQ_ACCEL_XOUT_L = 0x02
FXOS8700CQ_ACCEL_YOUT_H = 0x03
FXOS8700CQ_ACCEL_YOUT_L = 0x04
FXOS8700CQ_ACCEL_ZOUT_H = 0x05
FXOS8700CQ_ACCEL_ZOUT_L = 0x06
FXOS8700CQ_MAG_XOUT_H = 0x33
FXOS8700CQ_MAG_XOUT_L = 0x34
FXOS8700CQ_MAG_YOUT_H = 0x35
FXOS8700CQ_MAG_YOUT_L = 0x36
FXOS8700CQ_MAG_ZOUT_H = 0x37
FXOS8700CQ_MAG_ZOUT_L = 0x38
FXOS8700CQ_XYZ_DATA_CFG = 0x0E
FXOS8700CQ_STATUS = 0x00
FXOS8700CQ_MSTATUS = 0x32
FXOS8700CQ_WHO_AM_I = 0x0D
FXOS8700CQ_MOFF_X_LSB = 0x3f
FXOS8700CQ_MOFF_X_MSB = 0x40
FXOS8700CQ_MOFF_Y_LSB = 0x41
FXOS8700CQ_MOFF_Y_MSB = 0x42
FXOS8700CQ_MOFF_Z_LSB = 0x43
FXOS8700CQ_MOFF_Z_MSB = 0x44


# FXOS8700CQ Gyro

FXAS21002_I2C_ADDR = 0x21
FXAS21002_STATUS = 0x00
FXAS21002_OUT_X_MSB = 0x01
FXAS21002_OUT_X_LSB = 0x02
FXAS21002_OUT_Y_MSB = 0x03
FXAS21002_OUT_Y_LSB = 0x04
FXAS21002_OUT_Z_MSB = 0x05
FXAS21002_OUT_Z_LSB = 0x06
FXAS21002_DR_STATUS = 0x07
FXAS21002_F_STATUS = 0x08
FXAS21002_F_SETUP = 0x09
FXAS21002_F_EVENT = 0x0a
FXAS21002_INT_SRC_FLAG = 0x0b
FXAS21002_WHO_AM_I = 0x0c
FXAS21002_CTRL_REG_0 = 0x0d
FXAS21002_RT_CFG = 0x0e
FXAS21002_RT_SRC = 0x0f
FXAS21002_RT_THS = 0x10
FXAS21002_RT_COUNT = 0x11
FXAS21002_TEMP = 0x12
FXAS21002_CTRL_REG_1 = 0x13
FXAS21002_CTRL_REG_2 = 0x14
FXAS21002_CTRL_REG_3 = 0x15

MAG_UT_LSB = 0.1


def i2c_read_bytes(addr, reg, length):
    return [x for x in i2c_bus.read_i2c_block_data(addr, reg, length)]


def i2c_read(addr, reg):
    return i2c_bus.read_byte_data(addr, reg)


def i2c_write(addr, reg, val):
    i2c_bus.write_byte_data(addr, reg, val)


C216_MASK = (1 << 15)


def UINT8_TO_UINT16(h, l):
    return (h << 8 | l)


def C216_TO_INT16(a):
    return (-((a & C216_MASK) - (a & ~C216_MASK)))


def IMU_READ_TO_INT16(h, l):
    return (-((UINT8_TO_UINT16(h, l) & C216_MASK) - (UINT8_TO_UINT16(h, l) & ~C216_MASK)))


def read_mag():
    mxh, mxl, myh, myl, mzh, mzl = [x for x in i2c_bus.read_i2c_block_data(
        FXOS8700CQ_I2C_ADDR, FXOS8700CQ_MAG_XOUT_H, 6)]
    mx = IMU_READ_TO_INT16(int(mxh), int(mxl))
    my = IMU_READ_TO_INT16(int(myh), int(myl))
    mz = IMU_READ_TO_INT16(int(mzh), int(mzl))
    return mx, my, mz

def read_gyro():
    gxh, gxl, gyh, gyl, gzh, gzl = [x for x in i2c_bus.read_i2c_block_data(
        FXAS21002_I2C_ADDR, FXAS21002_OUT_X_MSB, 6)]
    gx = IMU_READ_TO_INT16(int(gxh), int(gxl))
    gy = IMU_READ_TO_INT16(int(gyh), int(gyl))
    gz = IMU_READ_TO_INT16(int(gzh), int(gzl))
    return gx, gy, gz

def read_acc():
    axh, axl, ayh, ayl, azh, azl = [x for x in i2c_bus.read_i2c_block_data(
        FXOS8700CQ_I2C_ADDR, FXOS8700CQ_ACCEL_XOUT_H, 6)]
    ax = IMU_READ_TO_INT16(int(axh), int(axl))
    ay = IMU_READ_TO_INT16(int(ayh), int(ayl))
    az = IMU_READ_TO_INT16(int(azh), int(azl))
    return ax, ay, az

def calibrate_mag():
    mx_avg = 0
    my_avg = 0
    mz_avg = 0
    mx_max = 0
    my_max = 0
    mz_max = 0
    mx_min = 0
    my_min = 0
    mz_min = 0
    i = 0

    DataReady = 0

    while (i < 94):  # This takes ~30s (94 samples * 1/3.125)
        # Compute 16-bit X-axis magnetic output value
        mx, my, mz = read_mag()
        #  Assign first sample to maximum and minimum values
        if (i == 0):
            mx_max = mx
            mx_min = mx
            my_max = my
            my_min = my
            mz_max = mz
            mz_min = mz

        #  Check to see if current sample is the maximum or minimum X-axis value
        if (mx > mx_max):
            mx_max = mx
        if (mx < mx_min):
            mx_min = mx

        #  Check to see if current sample is the maximum or minimum Y-axis value
        if (my > my_max):
            my_max = my
        if (my < my_min):
            my_min = my

        #  Check to see if current sample is the maximum or minimum Z-axis value
        if (mz > mz_max):
            mz_max = mz
        if (mz < mz_min):
            mz_min = mz

        i += 1
        # print("calib iter ", i)
        time.sleep(0.1)

    mx_avg = (mx_max + mx_min) / 2  # X-axis hard-iron offset
    my_avg = (my_max + my_min) / 2  # Y-axis hard-iron offset
    mz_avg = (mz_max + mz_min) / 2  # Z-axis hard-iron offset

    #  Left-shift by one as magnetometer offset registers are 15-bit only, left justified
    mx_avg = int(mx_avg) << 1
    my_avg = int(my_avg) << 1
    mz_avg = int(mz_avg) << 1
    print("Xoff: %.3f, Yoff=%.3f Zoff=%.3f" % (mx_avg, my_avg, mz_avg))
    # Standby mode to allow writing to the offset registers
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_1, 0x00)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_MOFF_X_LSB, (mx_avg & 0xFF))
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_MOFF_X_MSB,
              ((mx_avg >> 8) & 0xFF))
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_MOFF_Y_LSB, (my_avg & 0xFF))
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_MOFF_Y_MSB,
              ((my_avg >> 8) & 0xFF))
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_MOFF_Z_LSB, (mz_avg & 0xFF))
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_MOFF_Z_MSB,
              ((mz_avg >> 8) & 0xFF))
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_1,
              0x35)  # Active mode again


print("FXOS8700 Who am I: {}".format(hex(i2c_read(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_WHO_AM_I))))
print("FXAS21002 Who am I: {}".format(hex(i2c_read(FXAS21002_I2C_ADDR, FXAS21002_WHO_AM_I))))


def setup_mag():

    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_1,   0x00)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_XYZ_DATA_CFG, 0x00)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_2,   0x02)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_1,   0x15)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_M_CTRL_REG_1, 0x9f)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_M_CTRL_REG_2, 0x20)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_2,   0x02)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_3,   0x00)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_4,   0x01)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_5,   0x01)
    i2c_write(FXOS8700CQ_I2C_ADDR, FXOS8700CQ_CTRL_REG_1,   0x35)


def setup_gyro():
    ctrl_reg_1 = i2c_read(FXAS21002_I2C_ADDR,FXAS21002_CTRL_REG_1)
    i2c_write(FXAS21002_I2C_ADDR, FXAS21002_CTRL_REG_1, ctrl_reg_1 | 0x02)
    ctrl_reg_0 = i2c_read(FXAS21002_I2C_ADDR,FXAS21002_CTRL_REG_0)
    i2c_write(FXAS21002_I2C_ADDR,FXAS21002_CTRL_REG_0, ctrl_reg_0 | 0x03) # set gyro to full range scale 2000dps
    f_setup = i2c_read(FXAS21002_I2C_ADDR, FXAS21002_F_SETUP)
    i2c_write(FXAS21002_I2C_ADDR, FXAS21002_F_SETUP, f_setup | 0x40)
    ctrl_reg_3 = i2c_read(FXAS21002_I2C_ADDR, FXAS21002_CTRL_REG_3)
    i2c_write(FXAS21002_I2C_ADDR, FXAS21002_CTRL_REG_3,ctrl_reg_3 | 0x01)


setup_mag()

setup_gyro()

def serial_print(msg):
    global ser
    assert(type(msg) == str)
    for i in msg:
        ser.write(bytes(i, 'utf-8'))

# calibrate_mag()

while(1):
    mx, my, mz = read_mag()
    gx, gy, gz = read_gyro()
    ax, ay, az = read_acc()
    mx *= MAG_UT_LSB
    my *= MAG_UT_LSB
    mz *= MAG_UT_LSB
    gx *= 4000/2**16 # gyro is configured at 16 bits +-4000 dps scale range
    gy *= 4000/2**16 # gyro is configured at 16 bits +-4000 dps scale range
    gz *= 4000/2**16 # gyro is configured at 16 bits +-4000 dps scale range
    ax *= 8/2**16 # accel is configured at +-8g scale range
    ay *= 8/2**16 # accel is configured at +-8g scale range
    az *= 8/2**16 # accel is configured at +-8g scale range
    # print("ax = %.3f, ay = %.3f, az = %.3f mx = %.3f, my = %.3f, mz = %.3f gx = %.3f, gy = %.3f, gz = %.3f" % (ax, ay , az, mx, my, mz, gx, gy, gz))
    serial_print("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n" % (ax, ay , az, gx, gy, gz, mx, my, mz))
    # Mmag = math.sqrt(mx**2 + my**2 + mz**2)
    # alpha = math.acos(mx/Mmag) * 180 / math.pi
    # beta = math.acos(my/Mmag) * 180 / math.pi
    # gamma = math.acos(mz/Mmag) * 180 / math.pi
    # print("alpha=%.3f\tbeta=%.3f\tgamma=%.3f\tMag:%.3f" %
    #       (alpha, beta, gamma, Mmag))
