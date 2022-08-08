import numpy as np
import math
from params import *
from simple_pid import PID
from scipy.spatial.transform import Rotation as R
import cv2
from cv2 import aruco


class Sensor:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.imu = self.robot.getDevice("inertial unit")
        self.gyro = self.robot.getDevice("gyro")
        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")
        self.camera = self.robot.getDevice("camera")

    def enable(self, timestep):
        self.imu.enable(timestep)
        self.gyro.enable(timestep)
        self.gps.enable(timestep)
        self.compass.enable(timestep)
        self.camera.enable(timestep)

    def read_imu(self, show=False):
        self.roll = ((self.imu.getRollPitchYaw()[0] + math.pi / 2.0) * 180 / math.pi) - 90
        self.pitch = self.imu.getRollPitchYaw()[1] * 180 / math.pi
        self.yaw = self.imu.getRollPitchYaw()[2] * 180 / math.pi
        if show:
            print("roll={: .2f}|pitch={: .2f}|yaw={: .2f}".format(self.roll, self.pitch, self.yaw))
        return self.roll, self.pitch, self.yaw

    def read_gyro(self, show=False):
        self.roll_accel = self.gyro.getValues()[0] * 180 / math.pi
        self.pitch_accel = self.gyro.getValues()[1] * 180 / math.pi
        self.yaw_accel = self.gyro.getValues()[2] * 180 / math.pi
        if show:
            print(
                "roll_accel={: .2f}|pitch_accel={: .2f}|yaw_accel={: .2f}".format(
                    self.roll_accel, self.pitch_accel, self.yaw_accel
                )
            )
        return self.roll_accel, self.pitch_accel, self.yaw_accel

    def read_gps(self, show=False):
        self.xpos = self.gps.getValues()[0]
        self.ypos = self.gps.getValues()[1]
        self.zpos = self.gps.getValues()[2]
        if show:
            print("xpos={: .2f}|ypos={: .2f}|zpos={: .2f}".format(self.xpos, self.ypos, self.zpos))
        return self.xpos, self.ypos, self.zpos

    def read_compass(self, show=False):
        self.xcom = self.compass.getValues()[0]
        self.ycom = self.compass.getValues()[1]
        self.zcom = self.compass.getValues()[2]
        if show:
            print("xcom={: .2f}|ycom={: .2f}|zcom={: .2f}".format(self.xcom, self.ycom, self.zcom))
        return self.xcom, self.ycom, self.zcom

    def read_compass_head(self, show=False):
        self.compass_value = self.compass.getValues()
        self.heading = math.atan2(self.compass_value[0], self.compass_value[1]) - (math.pi / 2)
        if self.heading < -math.pi:
            self.heading = self.heading + (2 * math.pi)
        self.heading = self.heading * 180 / math.pi
        if show:
            print("heading={: .2f}".format(self.heading))
        return self.heading

    def read_camera(self):
        self.camera_height_width = self.read_camera_resolution()
        self.image = self.camera.getImage()
        self.image = np.frombuffer(self.image, np.uint8).reshape(
            (self.camera_height_width[0], self.camera_height_width[1], 4)
        )
        return self.image

    def read_camera_resolution(self):
        self.camera_height = self.camera.getHeight()
        self.camera_width = self.camera.getWidth()
        return self.camera_height, self.camera_width


class Actuator:
    def __init__(self, robot):
        self.robot = robot
        self.motor_fl = self.robot.getDevice("front left propeller")
        self.motor_fr = self.robot.getDevice("front right propeller")
        self.motor_rl = self.robot.getDevice("rear left propeller")
        self.motor_rr = self.robot.getDevice("rear right propeller")
        self.camera_roll = self.robot.getDevice("camera roll")
        self.camera_pitch = self.robot.getDevice("camera pitch")
        self.camera_yaw = self.robot.getDevice("camera yaw")

    def arming(self, arming_speed=0.0):
        self.motor_fl.setPosition(float("inf"))
        self.motor_fr.setPosition(float("inf"))
        self.motor_rl.setPosition(float("inf"))
        self.motor_rr.setPosition(float("inf"))
        self.motor_fl.setVelocity(arming_speed)
        self.motor_fr.setVelocity(arming_speed)
        self.motor_rl.setVelocity(arming_speed)
        self.motor_rr.setVelocity(arming_speed)

    def gimbal_control(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0):
        self.camera_roll.setPosition(roll_angle)
        self.camera_pitch.setPosition(pitch_angle)
        self.camera_yaw.setPosition(yaw_angle)

    def motor_speed(self, motor_fl=0, motor_fr=0, motor_rl=0, motor_rr=0, show=False):
        self.motor_fl.setVelocity(motor_fl)
        self.motor_fr.setVelocity(-motor_fr)
        self.motor_rl.setVelocity(-motor_rl)
        self.motor_rr.setVelocity(motor_rr)
        if show:
            print(
                "motor_fl={: .2f}|motor_fr={: .2f}|motor_rl={: .2f}|motor_rr={: .2f}".format(
                    motor_fl, -motor_fr, -motor_rl, motor_rr
                )
            )


class Marker:
    def __init__(self):
        self.radius = 3
        self.color_blue = (0, 0, 255)
        self.color_red = (255, 0, 0)
        self.thickness = 3

    def find_aruco(self, image):
        self.image = image
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.corner, self.id, self.reject = aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return self.corner, self.id, self.reject

    def get_center(self, cam_reso=[0, 0]):
        self.image_height, self.image_width = cam_reso
        # self.corner_lb = (int(self.corner[0][0][0][0]), int(self.corner[0][0][0][1]))
        # self.corner_lt = (int(self.corner[0][0][1][0]), int(self.corner[0][0][1][1]))
        # self.corner_rb = (int(self.corner[0][0][2][0]), int(self.corner[0][0][2][1]))
        # self.corner_rt = (int(self.corner[0][0][3][0]), int(self.corner[0][0][3][1]))
        self.center_x = (int(self.corner[0][0][3][0]) / 2) + (int(self.corner[0][0][0][0]) / 2)
        self.center_y = (int(self.corner[0][0][2][1]) / 2) + (int(self.corner[0][0][3][1]) / 2)
        return int(self.image_height / 2), int(self.image_width / 2), self.center_x, self.center_y

    def create_marker(self, xpos, ypos, radius=3, color=(255, 0, 0), thickness=1):
        self.image = cv2.circle(self.image, (int(xpos), int(ypos)), radius, color, thickness)
        return self.image
