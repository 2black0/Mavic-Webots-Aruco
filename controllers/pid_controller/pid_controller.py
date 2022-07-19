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

# definition the sensor
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
camera = robot.getDevice("camera")
camera.enable(timestep)

# definition the keyboard
keyboard = Keyboard()
keyboard.enable(timestep)

# definition the motor
motor_front_left = robot.getDevice("front left propeller")
motor_front_right = robot.getDevice("front right propeller")
motor_rear_left = robot.getDevice("rear left propeller")
motor_rear_right = robot.getDevice("rear right propeller")
motors = [motor_front_left, motor_front_right, motor_rear_left, motor_rear_right]
camera_roll = robot.getDevice("camera roll")
camera_pitch = robot.getDevice("camera pitch")
camera_yaw = robot.getDevice("camera yaw")

# camera face down
camera_roll.setPosition(0)
camera_pitch.setPosition(camera_down_angle)
camera_yaw.setPosition(0)


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
        return "sensor enable"

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
        self.ypos = self.gps.getValues()[1]
        self.zpos = self.gps.getValues()[2]
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

    def convert_to_attitude(self, x, y, yaw):
        self.c, self.s = np.cos(yaw), np.sin(yaw)
        self.R = np.array(((self.c, -self.s), (self.s, self.c)))
        self.converted = np.matmul([x, y], self.R)
        return self.converted


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
        return "arming"

    def gimbal_down(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0):
        self.camera_roll.setPosition(roll_angle)
        self.camera_pitch.setPosition(pitch_angle)
        self.camera_yaw.setPosition(yaw_angle)
        return "camera face down"

    def motor_speed(self, motor_fl=0, motor_fr=0, motor_rl=0, motor_rr=0):
        self.motor_fl.setVelocity(motor_fl)
        self.motor_fr.setVelocity(motor_fr)
        self.motor_rl.setVelocity(motor_rl)
        self.motor_rr.setVelocity(motor_rr)


class PDController:
    def __init__(
        self,
        roll_param,
        pitch_param,
        yaw_param,
        sampling_period,
        roll_target=0.0,
        pitch_target=0.0,
        yaw_target=0.0,
        z_target=2.0,
        # roll_old_error=0.0,
        # pitch_old_error=0.0,
        # yaw_old_error=0.0,
    ):
        self.roll_param = roll_param
        self.pitch_param = pitch_param
        self.yaw_param = yaw_param
        self.z_param = z_param
        self.sampling_period = sampling_period
        self.roll_target = roll_target
        self.pitch_target = pitch_target
        self.yaw_target = yaw_target
        self.z_target = z_target
        # self.roll_old_error = roll_old_error
        # self.pitch_old_error = pitch_old_error
        # self.yaw_old_error = yaw_old_error

    def attitude_calculation(self, imu=[0, 0, 0], gyro=[0, 0, 0], head=0):
        self.roll_error = self.roll_target - imu[0]
        self.pitch_error = self.pitch_target - imu[1]
        self.yaw_error = self.yaw_target - head
        # self.roll_derivative = (self.roll_error - self.roll_old_error) / self.sampling_period
        # self.roll_old_error = self.roll_error
        # self.pitch_derivative = (self.pitch_error - self.pitch_old_error) / self.sampling_period
        # self.pitch_old_error = self.pitch_error
        # self.yaw_derivative = (self.yaw_error - self.yaw_old_error) / self.sampling_period
        # self.yaw_old_error = self.yaw_error
        # self.roll_response = (self.roll_param[0] * self.roll_error) + (self.roll_param[2] * self.roll_derivative)
        # self.pitch_response = (self.pitch_param[0] * self.pitch_error) + (self.pitch_param[2] * self.pitch_derivative)
        # self.yaw_response = (self.yaw_param[0] * self.yaw_error) + (self.yaw_param[2] * self.yaw_derivative)
        self.roll_response = (self.roll_param[0] * self.roll_error) + (self.roll_param[2] * gyro[0])
        self.pitch_response = (self.pitch_param[0] * self.pitch_error) + (self.pitch_param[2] * gyro[1])
        self.yaw_response = (self.yaw_param[0] * self.yaw_error) + (self.yaw_param[2] * gyro[2])
        return (
            self.roll_error,
            self.pitch_error,
            self.yaw_error,
            self.roll_response,
            self.pitch_response,
            self.yaw_response,
        )

    def altitude_calculation(self, gps=[0, 0, 0]):
        self.z_error = gps[2] - self.z_target
        self.z_response = self.z_param[0] * self.z_error
        return self.z_error, self.z_response


sensor = Sensor(robot)
sensor.enable(timestep)
motor = Actuator(robot)
motor.arming(arming_speed=25)
motor.gimbal_down(pitch_angle=1.6)

while robot.step(timestep) != -1:
    imu = sensor.read_imu()
    # print("roll={: .2f} | pitch={: .2f} | yaw={: .2f}".format(imu[0], imu[1], imu[2]))
    gyro = sensor.read_gyro()
    # print("roll_accel={: .2f} | pitch_accel={: .2f} | yaw_accel={: .2f}".format(gyro[0], gyro[1], gyro[2]))
    gps = sensor.read_gps()
    # print("xpos={: .2f} | ypos={: .2f} | zpos={: .2f}".format(gps[0], gps[1], gps[2]))
    head = sensor.read_compass()
    # print("heading={: .2f}".format(head))
    image = sensor.read_camera()
    cv2.imshow("camera", image)
    cv2.waitKey(1)

    attitude_error = sensor.convert_to_attitude(x=gps[0], y=gps[1], yaw=head)
    # print("roll_error={: .2f} | pitch_error={: .2f}".format(attitude_error[1], attitude_error[0]))

    control = PDController(
        roll_param=roll_param,
        pitch_param=pitch_param,
        yaw_param=yaw_param,
        sampling_period=sampling_period,
        roll_target=roll_target,
        pitch_target=pitch_target,
        yaw_target=yaw_target,
        z_target=z_target,
    )
    attitude_calculation = control.attitude_calculation(imu=imu, gyro=gyro, head=head)
    """print(
        "roll_response={: .2f} | pitch_response={: .2f} | yaw_response={: .2f}".format(
            attitude_calculation[0], attitude_calculation[1], attitude_calculation[2]
        )
    )"""
    altitude_calculation = control.altitude_calculation(gps=gps)
    # print("z_response={: .2f}".format(altitude_calculation))

    print(
        "roll_error={: .2f} | pitch_error={: .2f} | yaw_error={: .2f} | z_error={: .2f} | roll_response={: .2f} | pitch_response={: .2f} | yaw_response={: .2f} | z_response{: .2f}".format(
            attitude_calculation[0],
            attitude_calculation[1],
            attitude_calculation[2],
            altitude_calculation[0],
            attitude_calculation[3],
            attitude_calculation[4],
            attitude_calculation[5],
            altitude_calculation[1],
        )
    )

    motor_fl = (
        take_off_pwm
        + altitude_calculation[1]
        - attitude_calculation[3]
        - attitude_calculation[4]
        + attitude_calculation[5]
    )
    motor_fr = (
        take_off_pwm
        + altitude_calculation[1]
        + attitude_calculation[3]
        - attitude_calculation[4]
        - attitude_calculation[5]
    )
    motor_rl = (
        take_off_pwm
        + altitude_calculation[1]
        - attitude_calculation[3]
        + attitude_calculation[4]
        - attitude_calculation[5]
    )
    motor_rr = (
        take_off_pwm
        + altitude_calculation[1]
        + attitude_calculation[3]
        + attitude_calculation[4]
        + attitude_calculation[5]
    )

    motor.motor_speed(motor_fl=motor_fl, motor_fr=motor_fr, motor_rl=motor_rl, motor_rr=motor_rr)
cv2.destroyAllWindows()
