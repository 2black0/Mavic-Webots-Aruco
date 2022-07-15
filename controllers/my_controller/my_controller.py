"""my_controller controller."""

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

camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")
camera_yaw_motor = robot.getDevice("camera yaw")

front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

param_roll = [50, 5, 10]
param_pitch = [30, 0, 0]
param_alti = [2, 0, 0]

set_point_roll = 0.0
set_point_pitch = 0.0
set_point_yaw = 0.0

set_point_x = 0.0
set_point_y = 0.0
set_point_alti = 3.5

int_err_roll = 0.0
int_err_pitch = 0.0
bef_err_alti = 0.0

take_off_pwm = 68.5
alti_pwm_offset = 0.6

MAX_INTEGRAL_ERROR = 4.0
MIN_INTEGRAL_ERROR = -4.0

MAX_PITCH_ANGLE = 0.5
MIN_PITCH_ANGLE = -0.5
MAX_ROLL_ANGLE = 0.5
MIN_ROLL_ANGLE = -0.5

for i in range(4):
    motors[i].setPosition(float("inf"))
    motors[i].setVelocity(1.0)
print("arming")

# camera face down
camera_pitch_motor.setPosition(1.6)
camera_yaw_motor.setPosition(0)
camera_roll_motor.setPosition(0)


def motor_action(frontLeftMotorSpeed, frontRightMotorSpeed, rearLeftMotorSpeed, rearRightMotorSpeed):
    front_left_motor.setVelocity(frontLeftMotorSpeed)
    front_right_motor.setVelocity(-frontRightMotorSpeed)
    rear_left_motor.setVelocity(-rearLeftMotorSpeed)
    rear_right_motor.setVelocity(rearRightMotorSpeed)


def convert_to_pitch_roll(ex, ey, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array(((c, -s), (s, c)))
    exy_ = np.matmul([ex, ey], R)
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


while robot.step(timestep) != -1:
    # read sensor
    roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]

    altitude = gps.getValues()[1]
    px = gps.getValues()[0]
    py = gps.getValues()[2]

    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    yaw_acceleration = gyro.getValues()[2]

    # calculate position error
    err_x = px - set_point_x
    err_y = py - set_point_y
    err_alti = set_point_alti - altitude

    # get error attitude from position error
    err_pitch, err_roll = convert_to_pitch_roll(err_x, err_y, yaw)

    int_err_pitch = int_err_pitch + err_pitch
    int_err_roll = int_err_roll + err_roll

    # stabilize the camera
    camera_pitch_motor.setPosition(np.clip(((-0.1 * pitch_acceleration) + 1.6), -0.5, 1.7))
    camera_roll_motor.setPosition(-0.115 * roll_acceleration)
    camera_yaw_motor.setPosition(-0.115 * yaw_acceleration)

    # calulate attitude pwm
    roll_pwm = param_roll[0] * np.clip(roll, -1.0, 1.0) + roll_acceleration + err_roll
    pitch_pwm = param_pitch[0] * np.clip(pitch, -1.0, 1.0) - pitch_acceleration - err_pitch
    yaw_pwm = 0.05 * (set_point_yaw - yaw)

    # pitch_pwm = (
    #    (param_pitch[0] * np.clip(pitch, -1.0, 1.0))
    #    # - (param_pitch[1] * np.clip(int_err_pitch, -1.0, 1.0))
    #    - (param_pitch[2] * pitch_acceleration)
    # )
    # roll_pwm = param_roll[0] * np.clip(roll, -1.0, 1.0) + roll_acceleration + err_roll
    # yaw_pwm = 0.05 * (set_point_yaw - yaw)

    # calculate altitude pwm
    clamped_difference_altitude = np.clip(err_alti + alti_pwm_offset, -1.0, 1.0)
    alti_pwm = param_alti[0] * math.pow(clamped_difference_altitude, 3.0)

    # pwm combination of each motor
    frontLeftMotorSpeed = take_off_pwm + alti_pwm - roll_pwm - pitch_pwm + yaw_pwm
    frontRightMotorSpeed = take_off_pwm + alti_pwm + roll_pwm - pitch_pwm - yaw_pwm
    rearLeftMotorSpeed = take_off_pwm + alti_pwm - roll_pwm + pitch_pwm - yaw_pwm
    rearRightMotorSpeed = take_off_pwm + alti_pwm + roll_pwm + pitch_pwm + yaw_pwm

    # action of motor
    motor_action(frontLeftMotorSpeed, frontRightMotorSpeed, rearLeftMotorSpeed, rearRightMotorSpeed)

    """
    print(
        "r_pwm:{:.2f} | p_pwm:{:.2f} | y_pwm:{:.2f} | c_alti:{:.2f} | a_pwm:{:.2f} | fl_motor:{:.2f} | fr_motor:{:.2f} | rl_motor:{:.2f} | rr_motor:{:.2f}".format(
            roll_pwm,
            pitch_pwm,
            yaw_pwm,
            clamped_difference_altitude,
            alti_pwm,
            frontLeftMotorSpeed,
            frontRightMotorSpeed,
            rearLeftMotorSpeed,
            rearRightMotorSpeed,
        )
    )
    """
    """
    print(
        "r:{:.2f} | p:{:.2f} | y:{:.2f} | X:{:.2f} | Y:{:.2f} | Z:{:.2f} | gr:{:.2f} | gp:{:.2f} | gy:{:.2f} | er:{:.2f} | ep:{:.2f} | eZ:{:.2f} | ex:{:.2f} | ey:{:.2f} | fl:{:.2f} | fr:{:.2f} | rl:{:.2f} | rr:{:.2f}".format(
            roll,
            pitch,
            yaw,
            px,
            py,
            altitude,
            roll_acceleration,
            pitch_acceleration,
            yaw_acceleration,
            err_roll,
            err_pitch,
            err_alti,
            err_x,
            err_y,
            frontLeftMotorSpeed,
            frontRightMotorSpeed,
            rearLeftMotorSpeed,
            rearRightMotorSpeed,
        )
    )
    """
    """
    print(
        "r:{: .2f} | p:{: .2f} | y:{: .2f} | X:{: .2f} | Y:{: .2f} | Z:{: .2f} | gr:{: .2f} | gp:{: .2f} | gy:{: .2f} | er:{: .2f} | ep:{: .2f} | eZ:{: .2f} | ex:{: .2f} | ey:{: .2f} | fl:{: .2f} | fr:{: .2f} | rl:{: .2f} | rr:{: .2f}".format(
            roll,
            pitch,
            yaw,
            px,
            py,
            altitude,
            roll_acceleration,
            pitch_acceleration,
            yaw_acceleration,
            err_roll,
            err_pitch,
            err_alti,
            err_x,
            err_y,
            frontLeftMotorSpeed,
            frontRightMotorSpeed,
            rearLeftMotorSpeed,
            rearRightMotorSpeed,
        )
    )"""

    # camera tutorial from this link
    # https://erebus.rcj.cloud/docs/tutorials/sensors/rgb-camera/
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    # findAruco(image, draw=False)

    # gray = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if len(corners) != 0:
        # print(corners[0][0][0])
        left_bottom = (corners[0][0][0][0], corners[0][0][0][1])
        left_top = (corners[0][0][1][0], corners[0][0][1][1])
        right_top = (corners[0][0][2][0], corners[0][0][2][1])
        right_bottom = (corners[0][0][3][0], corners[0][0][3][1])
        x_center = corners[0][0][1][0] + ((corners[0][0][2][0] - corners[0][0][1][0]) / 2)
        y_center = corners[0][0][3][1] + ((corners[0][0][2][1] - corners[0][0][3][1]) / 2)
        radius = 5
        color = (255, 0, 0)
        thickness = 2
        image = cv2.circle(image, left_bottom, radius, color, thickness)
        image = cv2.circle(image, left_top, radius, color, thickness)
        image = cv2.circle(image, right_top, radius, color, thickness)
        image = cv2.circle(image, right_bottom, radius, color, thickness)
    # aruco.drawDetectedMarkers(image, corners)

    # image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
    # frame_markers = aruco.drawDetectedMarkers(image, corners, ids)

    cv2.imshow("camera", image)
    cv2.waitKey(1)  # Render imshows on screen

cv2.destroyAllWindows()
# roll zero 0
# roll kiri negatif
# roll kanan positif

# pitch zero 0.07
# pitch up positif
# pitch down negatif

# yaw zero 0
# yaw cw negatif
# yaw ccw positif

# height zero 0.00
# naik positif
# turun negatif

# posisi X 0
# posisi X maju positif
# posisi X mundur negatif

# posisi Y 0
# posisi Y kiri negatif
# posisi Y kanan positif

# nitip link
# https://github.sre.pub/felipenmartins/Robotics-Simulation-Labs
