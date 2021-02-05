#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
import re
import math

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

ax, ay, az, mx, my, mz, gx, gy, gz = (0, 0, 0, 0, 0, 0, 0, 0, 0)

time_start, time_now, ts = (0, 0, 0.008)


class Angles:
    roll = 0
    pitch = 0
    jaw = 0


acc_angle = Angles()
gyro_angle = Angles()
comp_filter_angles = Angles()
yaw = 0

def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def drawText(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(
        textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(),
                 GL_RGBA, GL_UNSIGNED_BYTE, textData)


def draw_prism(ox=0.0, oy=0.0, oz=0.0):
    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0 + ox,  0.2 + oy, -1.0 + oz)
    glVertex3f(-1.0 + ox,  0.2 + oy, -1.0 + oz)
    glVertex3f(-1.0 + ox,  0.2 + oy,  1.0 + oz)
    glVertex3f(1.0 + ox,  0.2 + oy,  1.0 + oz)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0 + ox, -0.2 + oy,  1.0 + oz)
    glVertex3f(-1.0 + ox, -0.2 + oy,  1.0 + oz)
    glVertex3f(-1.0 + ox, -0.2 + oy, -1.0 + oz)
    glVertex3f(1.0 + ox, -0.2 + oy, -1.0 + oz)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0 + ox,  0.2 + oy, 1.0 + oz)
    glVertex3f(-1.0 + ox,  0.2 + oy, 1.0 + oz)
    glVertex3f(-1.0 + ox, -0.2 + oy, 1.0 + oz)
    glVertex3f(1.0 + ox, -0.2 + oy, 1.0 + oz)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0 + ox, -0.2 + oy, -1.0 + oz)
    glVertex3f(-1.0 + ox, -0.2 + oy, -1.0 + oz)
    glVertex3f(-1.0 + ox,  0.2 + oy, -1.0 + oz)
    glVertex3f(1.0 + ox,  0.2 + oy, -1.0 + oz)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0 + ox,  0.2 + oy,  1.0 + oz)
    glVertex3f(-1.0 + ox,  0.2 + oy, -1.0 + oz)
    glVertex3f(-1.0 + ox, -0.2 + oy, -1.0 + oz)
    glVertex3f(-1.0 + ox, -0.2 + oy,  1.0 + oz)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0 + ox,  0.2 + oy, -1.0 + oz)
    glVertex3f(1.0 + ox,  0.2 + oy,  1.0 + oz)
    glVertex3f(1.0 + ox, -0.2 + oy,  1.0 + oz)
    glVertex3f(1.0 + ox, -0.2 + oy, -1.0 + oz)
    glEnd()


def draw_and_rotate(rot_x, rot_y, rot_z):
    # the way I'm holding the IMU board, X and Y axis are switched
    # with respect to the OpenGL coordinate system
    glRotatef(rot_y, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(rot_x, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis
    glRotatef(rot_z, 0.0, 1.0, 0.0)  # Jaw,  rotate around z-axis

    draw_prism(0, 0, 0)

    glRotatef(-rot_z, 0.0, 1.0, 0.0)  # Jaw,  rotate around z-axis
    glRotatef(-rot_x, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis
    glRotatef(-rot_y, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis


def draw_imu_legend(x_offset, roll, pitch, yaw, data_name):
    drawText((x_offset, -2, 2), "{}:".format(data_name))
    drawText((x_offset, -2.3, 2), "Roll: {0:.2f}".format(roll))
    drawText((x_offset, -2.6, 2), "Pitch: {0:.2f}".format(pitch))
    drawText((x_offset, -2.9, 2), "Yaw: {0:.2f}".format(yaw))


def draw():
    global rquad, yaw
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    acc_angle.pitch = -1 * \
        (180/math.pi * math.atan2(ax, math.sqrt(ay**2 + az**2)))
    acc_angle.roll = 180/math.pi * math.atan2(ay, math.sqrt(ax**2 + az**2))
    gyro_angle.pitch += gy*ts
    gyro_angle.roll += gx*ts
    yaw += gz*ts
    # complementary filter values
    comp_filter_angles.pitch = gyro_angle.pitch*0.98 + acc_angle.pitch*0.02
    comp_filter_angles.roll = gyro_angle.roll*0.98 + acc_angle.roll*0.02
    glLoadIdentity()
    glTranslatef(0, 0.0, -15.0)

    acc_draw_offset = -6
    gyro_draw_offset = 0
    comp_draw_offset = 6

    draw_imu_legend(acc_draw_offset - 0.5, acc_angle.roll, acc_angle.pitch, yaw, 'Acc data')
    draw_imu_legend(gyro_draw_offset - 1, gyro_angle.roll, gyro_angle.pitch, yaw, 'Gyro data')
    draw_imu_legend(comp_draw_offset - 2, comp_filter_angles.roll, comp_filter_angles.pitch, yaw, 'Mag data')

    glTranslatef(acc_draw_offset, 0.0, 0.0)

    draw_and_rotate(acc_angle.roll, acc_angle.pitch, yaw)

    glTranslatef(-acc_draw_offset, 0.0, 0.0)

    draw_and_rotate(gyro_angle.roll, gyro_angle.pitch, yaw)

    glTranslatef(comp_draw_offset, 0.0, 0.0)

    draw_and_rotate(comp_filter_angles.roll, comp_filter_angles.pitch, yaw)


def read_data():
    global ax, ay, az, mx, my, mz, gx, gy, gz

    line = ser.readline()
    line = str(line)
    coords = re.findall(r'(-?\d+.\d+)', line)
    temp_ax = ax
    temp_ay = ay
    temp_az = az
    temp_gx = gx
    temp_gy = gy
    temp_gz = gz
    temp_mx = mx
    temp_my = my
    temp_mz = mz
    if len(coords) == 9:
        # Do exception handling in case the serial characters are nonsense
        try:
            # This is running on the pi mpu9250_master_example.py which make the calculations mentioned above before sending data
            ax = float(coords[0])
            ay = float(coords[1])
            az = float(coords[2])
            gx = float(coords[3])
            gy = float(coords[4])
            gz = float(coords[5])
            mx = float(coords[6])
            my = float(coords[7])
            mz = float(coords[8])

        # If the parsed ints are non sense, keep the previous values
        except:
            ax = temp_ax
            ay = temp_ay
            az = temp_az
            gx = temp_gx
            gy = temp_gy
            gz = temp_gz
            mx = temp_mx
            my = temp_my
            mz = temp_mz


def main():
    video_flags = OPENGL | DOUBLEBUF
    window_height = 600
    window_width = 800
    pygame.init()
    screen = pygame.display.set_mode(
        (window_width, window_height), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(window_width, window_height)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (
                event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        read_data()
        draw()

        pygame.display.flip()
        frames = frames + 1

    print("fps:  %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
    ser.close()


if __name__ == '__main__':
    main()
