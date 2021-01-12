#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
import re
import math

ser = serial.Serial('COM7', 115200, timeout=1)

ax_acc, ay_acc = (0.0, 0.0)
ax_gyro, ay_gyro = (0.0, 0.0)
ax_comp, ay_comp = (0.0, 0.0)
az = 0.0


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
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()
    glTranslatef(0, 0.0, -15.0)

    acc_draw_offset = -6
    gyro_draw_offset = 0
    comp_draw_offset = 6

    draw_imu_legend(acc_draw_offset - 0.5, ax_acc, ay_acc, az, 'Acc data')
    draw_imu_legend(gyro_draw_offset - 1, ax_gyro, ay_gyro, az, 'Gyro data')
    draw_imu_legend(comp_draw_offset - 2, ax_comp, ay_comp, az, 'Comp filter')


    glTranslatef(acc_draw_offset, 0.0, 0.0)

    draw_and_rotate(ax_acc, ay_acc, az)

    glTranslatef(-acc_draw_offset, 0.0, 0.0)

    draw_and_rotate(ax_gyro, ay_gyro, az)

    glTranslatef(comp_draw_offset, 0.0, 0.0)

    draw_and_rotate(ax_comp, ay_comp, az)


def read_data():
    global ax_acc, ay_acc, ax_gyro, ay_gyro, ax_comp, ay_comp, az

    line = ser.readline()
    line = str(line)
    coords = re.findall(r'(-?\d+.\d+)', line)
    temp_ax_acc = ax_acc
    temp_ay_acc = ay_acc
    temp_ax_gyro = ax_gyro
    temp_ay_gyro = ay_gyro
    temp_ax_comp = ax_comp
    temp_ay_comp = ay_comp
    temp_az = az
    if len(coords) == 7:
        # Do exception handling in case the serial characters are nonsense
        try:
            # This is running on the pi mpu9250_master_example.py which make the calculations mentioned above before sending data
            ax_acc = float(coords[0])
            ay_acc = float(coords[1])
            ax_gyro = float(coords[2])
            ay_gyro = float(coords[3])
            ax_comp = float(coords[4])
            ay_comp = float(coords[5])
            az = float(coords[6])
        # If the parsed ints are non sense, keep the previous values
        except:
            ax_acc = temp_ax_acc
            ay_acc = temp_ay_acc
            ax_gyro = temp_ax_gyro
            ay_gyro = temp_ay_gyro
            ax_comp = temp_ax_comp
            ay_comp = temp_ay_comp
            az = temp_az


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
