import numpy as np
import math
from params import *


class Sensor:
    def __init__(self, robot):
        # print("sensor init")
        self.robot = robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.imu = self.robot.getDevice("inertial unit")
        self.gyro = self.robot.getDevice("gyro")
        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")
        self.camera = self.robot.getDevice("camera")

    def enable(self, timestep):
        # print("sensor enable")
        self.imu.enable(timestep)
        self.gyro.enable(timestep)
        self.gps.enable(timestep)
        self.compass.enable(timestep)
        self.camera.enable(timestep)

    def read_imu(self, show=False):
        self.roll = (self.imu.getRollPitchYaw()[0] + math.pi / 2.0) * 180 / math.pi
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
        self.ypos = self.gps.getValues()[2]
        self.zpos = self.gps.getValues()[1]
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
        # self.camera_width = self.camera.getWidth()
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

    def gimbal_down(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0):
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


class Controller:
    def __init__(
        self,
        roll_param,
        pitch_param,
        yaw_param,
        z_param,
        z_takeoff=68.5,
        # z_takeoff=75,
        # z_offset=0.6,
        z_offset=0.0,
    ):
        self.roll_param = roll_param
        self.pitch_param = pitch_param
        self.yaw_param = yaw_param
        self.z_param = z_param
        self.z_takeoff = z_takeoff
        self.z_offset = z_offset

    def error_calculation(self, target=[0, 0, 0, 0], gps=[0, 0, 0], marker=[0, 0, 0, 0], head=0, status=False):
        self.x_target = target[0]
        self.y_target = target[1]
        self.z_target = target[2]
        self.yaw_target = target[3]
        self.z_error = self.z_target - gps[2]
        if status:
            if marker[1] != 0:
                self.y_error = (marker[2] - marker[1]) / (marker[1] / 3)
                self.x_error = -(marker[3] - marker[0]) / (marker[0] / 3)
            else:
                self.x_error = 0
                self.y_error = 0
        else:
            self.x_error = gps[0] - self.x_target
            self.y_error = gps[1] - self.y_target

        self.yaw_error = self.yaw_target - head
        # print(head)
        return self.x_error, self.y_error, self.z_error, self.yaw_error

    def convert_to_attitude(self, x_error, y_error, yaw):
        self.c, self.s = np.cos(yaw), np.sin(yaw)
        self.R = np.array(((self.c, -self.s), (self.s, self.c)))
        self.converted = np.matmul([x_error, y_error], self.R)
        return self.converted

    def calculate(self, imu=[0, 0, 0], gyro=[0, 0, 0], error=[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]], head=0):
        self.x_error = error[0]
        self.y_error = error[1]
        self.z_error = error[2]
        self.yaw_error = error[3]

        self.pitch_error, self.roll_error = self.convert_to_attitude(
            np.clip(self.x_error[0], -1.5, 1.5), np.clip(self.y_error[0], -1.5, 1.5), head
        )

        self.roll_input = (
            (self.roll_param[0] * np.clip(imu[0], -0.5, 0.5)) + (self.roll_param[2] * gyro[0]) + self.roll_error
        )
        self.pitch_input = (
            (self.pitch_param[0] * np.clip(imu[1], -0.5, 0.5)) - (self.pitch_param[2] * gyro[1]) - self.pitch_error
        )

        self.yaw_input = (self.yaw_param[0] * self.yaw_error[0]) - (self.yaw_param[2] * self.yaw_error[2])

        self.z_param = thro_param
        self.z_input = (
            (self.z_param[0] * self.z_error[0])
            + (self.z_param[1] * self.z_error[1])
            + (self.z_param[2] * self.z_error[2])
        )

        return self.z_takeoff, self.z_input, self.roll_input, self.pitch_input, self.yaw_input

    def gimbal_control(self, gyro=[0, 0, 0], roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0):
        pitch_gimbal = np.clip(((-0.1 * gyro[1]) + pitch_angle), -0.5, 1.7)
        roll_gimbal = np.clip((-0.115 * gyro[0] + roll_angle), -0.5, 0.5)
        yaw_gimbal = np.clip((-0.115 * gyro[2] + yaw_angle), -1.7, 1.7)
        return roll_gimbal, pitch_gimbal, yaw_gimbal
