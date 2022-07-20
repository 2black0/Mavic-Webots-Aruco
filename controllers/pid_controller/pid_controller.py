"""pid_controller controller"""

# get the variable
from var import *

# declaration library
from controller import Robot
from controller import Keyboard
from controller import Motor
from controller import Camera
from controller import Gyro
from controller import Compass
from controller import InertialUnit
from controller import GPS
from controller import LED

import numpy as np
import math
import cv2, PIL

# instance
robot = Robot()

# time step
timestep = int(robot.getBasicTimeStep())


class Sensor:
    def __init__(self, robot):
        self.robot = robot
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

    def read_imu(self):
        self.roll = self.imu.getRollPitchYaw()[0] + math.pi / 2.0
        self.pitch = self.imu.getRollPitchYaw()[1]
        self.yaw = self.imu.getRollPitchYaw()[2]
        return self.roll, self.pitch, self.yaw

    def read_gyro(self):
        self.roll_accel = self.gyro.getValues()[0]
        self.pitch_accel = self.gyro.getValues()[1]
        self.yaw_accel = self.gyro.getValues()[2]
        return self.roll_accel, self.pitch_accel, self.yaw_accel

    def read_gps(self):
        self.xpos = self.gps.getValues()[0]
        self.ypos = self.gps.getValues()[2]
        self.zpos = self.gps.getValues()[1]
        return self.xpos, self.ypos, self.zpos

    def read_compass(self):
        self.compass_value = self.compass.getValues()
        self.heading = math.atan2(self.compass_value[0], self.compass_value[1]) - (math.pi / 2)
        if self.heading < -math.pi:
            self.heading = self.heading + (2 * math.pi)
        return self.heading

    def read_camera(self):
        self.image = self.camera.getImage()
        self.image = np.frombuffer(self.image, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
        return self.image


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

    def arming(self, arming_speed=5.0):
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

    def motor_speed(self, motor_fl=0, motor_fr=0, motor_rl=0, motor_rr=0):
        self.motor_fl.setVelocity(motor_fl)
        self.motor_fr.setVelocity(motor_fr)
        self.motor_rl.setVelocity(motor_rl)
        self.motor_rr.setVelocity(motor_rr)


class Controller:
    def __init__(
        self,
        roll_param,
        pitch_param,
        yaw_param,
        z_param,
        yaw_target=0.0,
        x_target=0.0,
        y_target=0.0,
        z_target=3.0,
        z_takeoff=68.5,
        z_offset=0.6,
    ):
        self.roll_param = roll_param
        self.pitch_param = pitch_param
        self.yaw_param = yaw_param
        self.z_param = z_param
        self.yaw_target = yaw_target
        self.x_target = x_target
        self.y_target = y_target
        self.z_target = z_target
        self.z_takeoff = z_takeoff
        self.z_offset = z_offset

    def convert_to_attitude(self, x_error, y_error, yaw):
        self.c, self.s = np.cos(yaw), np.sin(yaw)
        self.R = np.array(((self.c, -self.s), (self.s, self.c)))
        self.converted = np.matmul([x_error, y_error], self.R)
        return self.converted

    def calculate(self, imu=[0, 0, 0], gyro=[0, 0, 0], gps=[0, 0, 0], head=0):
        self.x_error = gps[0] - self.x_target
        self.y_error = gps[1] - self.y_target
        self.z_error = self.z_target - gps[2]
        self.pitch_error, self.roll_error = self.convert_to_attitude(self.x_error, self.y_error, head)

        self.roll_input = self.roll_param[0] * np.clip(imu[0], -0.5, 0.5) + gyro[0] + self.roll_error
        self.pitch_input = self.pitch_param[0] * np.clip(imu[1], -0.5, 0.5) - gyro[1] - self.pitch_error
        self.yaw_input = self.yaw_param[0] * (self.yaw_target - head)
        self.z_diff = np.clip(self.z_error + self.z_offset, -1.0, 1.0)
        self.z_input = self.z_param[0] * math.pow(self.z_diff, 3.0)
        return self.z_takeoff, self.z_input, self.roll_input, self.pitch_input, self.yaw_input


sensor = Sensor(robot)
sensor.enable(timestep)
motor = Actuator(robot)
motor.arming(arming_speed=25)
motor.gimbal_down(pitch_angle=1.6)

while robot.step(timestep) != -1:
    imu = sensor.read_imu()
    gyro = sensor.read_gyro()
    gps = sensor.read_gps()
    head = sensor.read_compass()
    # image = sensor.read_camera()

    # print("roll={: .2f} | pitch={: .2f} | yaw={: .2f}".format(imu[0], imu[1], imu[2]))
    # print("roll_accel={: .2f} | pitch_accel={: .2f} | yaw_accel={: .2f}".format(gyro[0], gyro[1], gyro[2]))
    # print("xpos={: .2f} | ypos={: .2f} | zpos={: .2f}".format(gps[0], gps[1], gps[2]))
    # print("heading={: .2f}".format(head))

    # cv2.imshow("camera", image)
    # cv2.waitKey(1)

    controller = Controller(
        roll_param=roll_param,
        pitch_param=pitch_param,
        yaw_param=yaw_param,
        z_param=z_param,
        yaw_target=0.0,
        x_target=0.0,
        y_target=0.0,
        z_target=3.0,
        z_takeoff=68.5,
        z_offset=0.6,
    )

    action = controller.calculate(imu=imu, gyro=gyro, gps=gps, head=head)

    motor_fl = action[0] + action[1] - action[2] - action[3] + action[4]
    motor_fr = action[0] + action[1] + action[2] - action[3] - action[4]
    motor_rl = action[0] + action[1] - action[2] + action[3] - action[4]
    motor_rr = action[0] + action[1] + action[2] + action[3] + action[4]

    motor.motor_speed(motor_fl=motor_fl, motor_fr=motor_fr, motor_rl=motor_rl, motor_rr=motor_rr)

    print(
        "z_in = {: .2f} | roll_in = {: .2f} | pitch_in = {: .2f} | yaw_in = {: .2f} | motor_fl = {: .2f} | motor_fr = {: .2f} | motor_rl = {: .2f} | motor_rr = {: .2f}".format(
            action[1], action[2], action[3], action[4], motor_fl, motor_fr, motor_rl, motor_rr
        )
    )
# cv2.destroyAllWindows()
