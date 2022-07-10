"""controller controller."""

from controller import (
    Robot,
    Motor,
    Camera,
    InertialUnit,
    Gyro,
    Compass,
    GPS,
    LED,
    Keyboard,
)

import math
import numpy as np
import struct

robot = Robot()

timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
camera = robot.getDevice("camera")
camera.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")
front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

receiver = robot.getDevice("receiver")
receiver.enable(timestep)

while robot.step(timestep) != -1:
    message = receiver.getData()

    roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]

    altitude = gps.getValues()[1]
    px = gps.getValues()[0]
    py = gps.getValues()[2]

    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]

    print(
        "r:{:.2f} | p:{:.2f} | y:{:.2f} | X:{:.2f} | Y:{:.2f} | Z:{:.2f} | gr:{:.2f} | gp:{:.2f}".format(
            roll, pitch, yaw, px, py, altitude, roll_acceleration, pitch_acceleration
        )
    )
