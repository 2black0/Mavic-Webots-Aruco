"""my_controller controller."""

from var import *
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
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# definition the hardware
keyboard = Keyboard()
keyboard.enable(timestep)

imu = robot.getDevice("inertial unit")
imu.enable(timestep)

camera = robot.getDevice("camera")
camera.enable(timestep)
# camera.recognitionEnable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

camera_roll = robot.getDevice("camera roll")
camera_pitch = robot.getDevice("camera pitch")
camera_yaw = robot.getDevice("camera yaw")

motor_front_left = robot.getDevice("front left propeller")
motor_front_right = robot.getDevice("front right propeller")
motor_rear_left = robot.getDevice("rear left propeller")
motor_rear_right = robot.getDevice("rear right propeller")
motors = [motor_front_left, motor_front_right, motor_rear_left, motor_rear_right]

# camera face down
camera_roll.setPosition(0)
camera_pitch.setPosition(camera_down_angle)
camera_yaw.setPosition(0)


def motor_action(speed_motor_front_left, speed_motor_front_right, speed_motor_rear_left, speed_motor_rear_right):
    motor_front_left.setVelocity(speed_motor_front_left)
    motor_front_right.setVelocity(-speed_motor_front_right)
    motor_rear_left.setVelocity(-speed_motor_rear_left)
    motor_rear_right.setVelocity(speed_motor_rear_right)


def convert_to_pitch_roll(x_error, y_error, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array(((c, -s), (s, c)))
    exy_ = np.matmul([x_error, y_error], R)
    # print("ex = %f, ey = %f" % (ex, ey))
    # print(yaw)
    # print("ex_ = %f, ey_ = %f" % (exy_[0], exy_[1]))
    return exy_[0], exy_[1]


# tutorial https://www.youtube.com/watch?v=AQXLC2Btag4
def findAruco(img, marker_size=6, total_markers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f"DICT_{marker_size}X{marker_size}_{total_markers}")
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, _ = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    if draw:
        aruco.drawDetectedMarkers(img, bbox)
    return [bbox, ids]


def read_imu(imu):
    roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]

    return roll, pitch, yaw


def read_gps(gps):
    alti = gps.getValues()[1]
    x = gps.getValues()[0]
    y = gps.getValues()[2]

    return alti, x, y


def read_gyro(gyro):
    roll_gyro = gyro.getValues()[0]
    pitch_gyro = gyro.getValues()[1]
    yaw_gyro = gyro.getValues()[2]

    return roll_gyro, pitch_gyro, yaw_gyro


def read_aruco(camera):
    # camera tutorial from this link
    # https://erebus.rcj.cloud/docs/tutorials/sensors/rgb-camera/
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

    # gray = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    image = cv2.circle(image, (int(camera_res_width / 2), int(camera_res_height / 2)), radius, color_center, thickness)
    x_center = 0
    y_center = 0
    status = 0
    if len(corners) != 0:
        status = 1
        # print(corners[0][0][0])
        # print(type(corners))

        left_bottom = (int(corners[0][0][0][0]), int(corners[0][0][0][1]))
        left_top = (int(corners[0][0][1][0]), int(corners[0][0][1][1]))
        right_top = (int(corners[0][0][2][0]), int(corners[0][0][2][1]))
        right_bottom = (int(corners[0][0][3][0]), int(corners[0][0][3][1]))
        x_center = (int(corners[0][0][3][0]) / 2) + (int(corners[0][0][0][0]) / 2)
        y_center = (int(corners[0][0][2][1]) / 2) + (int(corners[0][0][3][1]) / 2)
        # print(int(x_center), int(y_center))
        # image = cv2.circle(image, left_bottom, radius, color, thickness)
        # image = cv2.circle(image, left_top, radius, color, thickness)
        # image = cv2.circle(image, right_top, radius, color, thickness)
        # image = cv2.circle(image, right_bottom, radius, color, thickness)
        # image = cv2.circle(image, (int(x_center), int(y_center)), radius, color_pos, thickness)

    # aruco.drawDetectedMarkers(image, corners)

    # image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
    # frame_markers = aruco.drawDetectedMarkers(image, corners, ids)
    return status, image, x_center, y_center
    # cv2.imshow("camera", image)
    # cv2.waitKey(1)  # Render imshows on screen


# arming
for i in range(4):
    motors[i].setPosition(float("inf"))
    motors[i].setVelocity(1.0)
print("Arming")

while robot.step(timestep) != -1:
    roll, pitch, yaw = read_imu(imu)
    alti, x, y = read_gps(gps)
    roll_gyro, pitch_gyro, yaw_gyro = read_gyro(gyro)

    # read aruco marker
    status, image, x_center, y_center = read_aruco(camera)
    if status:
        image = cv2.circle(image, (int(x_center), int(y_center)), radius, color_pos, thickness)
    cv2.imshow("camera", image)
    cv2.waitKey(1)

    # calculate error based on aruco position
    x_aruco_error = ((camera_res_width / 2) - x_center) / (camera_res_width / 2)
    y_aruco_error = ((camera_res_height / 2) - y_center) / (camera_res_height / 2)

    # calculate position error
    x_error = x - x_setpoint
    y_error = y - y_setpoint
    alti_error = alti_setpoint - alti

    # get error attitude from position error
    if status:
        print("X error={: .2f} | Y error={: .2f}".format(y_aruco_error, x_aruco_error))
        pitch_error, roll_error = convert_to_pitch_roll(x_aruco_error, y_aruco_error, yaw)
    else:
        print("X error={: .2f}  | Y error={: .2f}".format(x_error, y_error))
        pitch_error, roll_error = convert_to_pitch_roll(x_error, y_error, yaw)

    pitch_interror = pitch_interror + pitch_error
    roll_interror = roll_interror + roll_error

    # stabilize the camera
    camera_pitch.setPosition(np.clip(((-0.1 * pitch_gyro) + camera_down_angle), -0.5, 1.7))
    camera_roll.setPosition(np.clip((-0.115 * roll_gyro), -0.5, 0.5))
    camera_yaw.setPosition(np.clip((-0.115 * yaw_gyro), -1.7, 1.7))

    # calulate attitude pwm
    roll_pwm = roll_parameter[0] * np.clip(roll, -1.0, 1.0) + roll_gyro + roll_error
    pitch_pwm = pitch_parameter[0] * np.clip(pitch, -1.0, 1.0) - pitch_gyro - pitch_error
    yaw_pwm = 0.05 * (yaw_setpoint - yaw)

    # calculate altitude pwm
    clamped_difference_altitude = np.clip(alti_error + alti_pwm_offset, -1.0, 1.0)
    alti_pwm = alti_parameter[0] * math.pow(clamped_difference_altitude, 3.0)

    # pwm combination of each motor
    frontLeftMotorSpeed = take_off_pwm + alti_pwm - roll_pwm - pitch_pwm + yaw_pwm
    frontRightMotorSpeed = take_off_pwm + alti_pwm + roll_pwm - pitch_pwm - yaw_pwm
    rearLeftMotorSpeed = take_off_pwm + alti_pwm - roll_pwm + pitch_pwm - yaw_pwm
    rearRightMotorSpeed = take_off_pwm + alti_pwm + roll_pwm + pitch_pwm + yaw_pwm

    # action of motor
    motor_action(frontLeftMotorSpeed, frontRightMotorSpeed, rearLeftMotorSpeed, rearRightMotorSpeed)

cv2.destroyAllWindows()
