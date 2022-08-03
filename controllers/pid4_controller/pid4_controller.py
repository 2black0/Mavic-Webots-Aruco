"""pid4_controller controller."""

from controller import Robot, Keyboard
from mavic_toolkit import Sensor, Actuator
from time import sleep
from params import *
from simple_pid import PID

robot = Robot()
timestep = int(robot.getBasicTimeStep())
sensor = Sensor(robot)
sensor.enable(timestep)
motor = Actuator(robot)
motor.arming(20.0)
keyboard = Keyboard()
keyboard.enable(timestep)

xPID = PID(float(roll_param[0]), float(roll_param[1]), float(roll_param[2]), setpoint=0.0)
yPID = PID(float(pitch_param[0]), float(pitch_param[1]), float(pitch_param[2]), setpoint=0.0)
zPID = PID(float(thro_param[0]), float(thro_param[1]), float(thro_param[2]), setpoint=0.0)
yawPID = PID(float(yaw_param[0]), float(yaw_param[1]), float(yaw_param[2]), setpoint=0.0)

xPID.output_limits = (-10, 10)
yPID.output_limits = (-10, 10)
zPID.output_limits = (-5, 5)
yawPID.output_limits = (-5, 5)

while robot.step(timestep) != -1:
    roll, pitch, yaw = sensor.read_imu()
    roll_accel, pitch_accel, yaw_accel = sensor.read_gyro()
    xpos, ypos, zpos = sensor.read_gps()
    head = sensor.read_compass_head()

    key = keyboard.getKey()
    while key > 0:
        if key == ord("T") and status_takeoff == False:
            status_takeoff = True
            status_landing = False
            z_target = 3.0
            print("Arming and Take Off")
            sleep(0.25)
            break

    xPID.setpoint = x_target
    yPID.setpoint = y_target
    zPID.setpoint = z_target
    yawPID.setpoint = yaw_target

    roll_error = xPID(xpos)
    pitch_error = yPID(ypos)

    vertical_input = zPID(zpos)
    roll_input = (roll_input_param[0] * roll) + roll_accel + roll_error
    pitch_input = (pitch_input_param[0] * pitch) - pitch_accel - pitch_error
    yaw_input = yawPID(head)

    motor_fl = vertical_thrust + vertical_input - roll_input - pitch_input - yaw_input
    motor_fr = vertical_thrust + vertical_input + roll_input - pitch_input + yaw_input
    motor_rl = vertical_thrust + vertical_input - roll_input + pitch_input + yaw_input
    motor_rr = vertical_thrust + vertical_input + roll_input + pitch_input - yaw_input

    motor.motor_speed(motor_fl=motor_fl, motor_fr=motor_fr, motor_rl=motor_rl, motor_rr=motor_rr)
