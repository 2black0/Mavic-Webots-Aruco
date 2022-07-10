"""uav_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import (
    Robot,
    Camera,
    InertialUnit,
    GPS,
    Compass,
    Gyro,
    Motor,
    Keyboard,
    Receiver,
)
import math
import numpy as np
import struct


def suggest_to_gain(params_dict):
    k_vertical_thrust = params_dict["k_vertical_thrust"]
    k_vertical_offset = params_dict["k_vertical_offset"]
    k_vertical_p = params_dict["k_vertical_p"]
    k_roll_p = params_dict["k_roll_p"]
    k_pitch_p = params_dict["k_pitch_p"]
    return k_vertical_thrust, k_vertical_offset, k_vertical_p, k_roll_p, k_pitch_p


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
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
emitter = robot.getDevice("emitter")

receiver.enable(timestep)

# bayes opt
# initial gain
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

reward = 0
# arming
for i in range(4):
    motors[i].setPosition(float("inf"))
    motors[i].setVelocity(1.0)
print("arming")

# target
target_z = 1
target_x = 0
target_y = 0
target_yaw = 0


def convert_to_pitch_roll(ex, ey, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array(((c, -s), (s, c)))
    exy_ = np.matmul([ex, ey], R)
    # print("ex = %f, ey = %f" % (ex, ey))
    # print(yaw)
    # print("ex_ = %f, ey_ = %f" % (exy_[0], exy_[1]))
    return exy_[0], exy_[1]


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    message = receiver.getData()
    paramList = struct.unpack("5f", message)
    receiver.nextPacket()
    k_vertical_thrust, k_vertical_offset, k_vertical_p, k_roll_p, k_pitch_p = (
        paramList[0],
        paramList[1],
        paramList[2],
        paramList[3],
        paramList[4],
    )

    roll = imu.getRollPitchYaw()[0] + math.pi / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    altitude = gps.getValues()[1]
    px = gps.getValues()[0]
    py = gps.getValues()[2]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]

    err_x = px - target_x
    err_y = target_y - py
    err_z = target_z - altitude

    # target for bayesopt

    # Compute the roll, pitch, yaw and vertical inputs.
    pitch_err, roll_err = convert_to_pitch_roll(err_x, err_y, yaw)
    # roll_input = k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acceleration + 1*(target_y - py)
    # pitch_input = k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acceleration + 20*(px - target_x)
    roll_input = k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acceleration - roll_err
    pitch_input = k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acceleration - pitch_err
    yaw_input = 0.1 * (target_yaw - yaw)
    clamped_difference_altitude = np.clip(err_z + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * math.pow(clamped_difference_altitude, 3.0)
    # Actuate the motors taking into consideration all the computed inputs.
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input

    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
