"""pid3_controller controller."""

from controller import Robot, Keyboard
from mavic_toolkit import Sensor, Actuator, Controller
from time import sleep
import cv2
from simple_pid import PID
from params import *

robot = Robot()
timestep = int(robot.getBasicTimeStep())
sensor = Sensor(robot)
sensor.enable(timestep)
motor = Actuator(robot)
motor.arming(5.0)
keyboard = Keyboard()
keyboard.enable(timestep)

rollPID = PID(float(roll_param[0]), float(roll_param[1]), float(roll_param[2]), setpoint=0.0)
pitchPID = PID(float(pitch_param[0]), float(pitch_param[1]), float(pitch_param[2]), setpoint=0.0)
yawPID = PID(float(yaw_param[0]), float(yaw_param[1]), float(yaw_param[2]), setpoint=0.0)
throttlePID = PID(float(thro_param[0]), float(thro_param[1]), float(thro_param[2]), setpoint=0.0)

rollPID.output_limits = (-10, 10)
pitchPID.output_limits = (-10, 10)
yawPID.output_limits = (-5, 5)
throttlePID.output_limits = (-5, 5)

while robot.step(timestep) != -1:
    roll, pitch, yaw = sensor.read_imu()
    roll_accel, pitch_accel, yaw_accel = sensor.read_gyro()
    xpos, ypos, zpos = sensor.read_gps()
    head = sensor.read_compass_head()
    # image = sensor.read_camera()

    key = keyboard.getKey()
    while key > 0:
        if key == ord("T") and status_takeoff == False:
            status_takeoff = True
            status_landing = False
            motor.arming(arming_speed=10.0)
            # motor.gimbal_down(pitch_angle=1.6)
            z_target = 3.0
            print("Arming and Take Off")
            sleep(0.25)
            break

    throttlePID.setpoint = z_target
    yawPID.setpoint = yaw_target
    rollPID.setpoint = x_target
    pitchPID.setpoint = y_target

    roll_error = rollPID(xpos)
    pitch_error = pitchPID(ypos)

    vertical_input = throttlePID(zpos)
    roll_input = (roll_input_param[0] * roll) + roll_accel - roll_error
    pitch_input = (pitch_input_param[0] * pitch) - pitch_accel + pitch_error
    yaw_input = yawPID(head)

    # print("roll_error={: .2f}|pitch_error={: .2f}|yaw_input={: .2f}".format(roll_error, pitch_error, yaw_input))
    print(
        "vertical_input={: .2f}|roll_input={: .2f}|pitch_input={: .2f}".format(vertical_input, roll_input, pitch_input)
    )

    motor_fl = vertical_thrust + vertical_input - roll_input - pitch_input - yaw_input
    motor_fr = vertical_thrust + vertical_input + roll_input - pitch_input + yaw_input
    motor_rl = vertical_thrust + vertical_input - roll_input + pitch_input + yaw_input
    motor_rr = vertical_thrust + vertical_input + roll_input + pitch_input - yaw_input

    # motor_fl = motor_fr = motor_rl = motor_rr = 0.0

    if status_takeoff == False and status_landing == False:
        motor_fl = motor_fr = motor_rl = motor_rr = 0.0

    motor.motor_speed(motor_fl=motor_fl, motor_fr=motor_fr, motor_rl=motor_rl, motor_rr=motor_rr)
