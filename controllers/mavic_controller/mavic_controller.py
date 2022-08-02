"""mavic_controller controller."""

from controller import Robot, Keyboard
from mavic_toolkit import Sensor, Actuator, Controller
from time import sleep
import cv2
from var import *

robot = Robot()
timestep = int(robot.getBasicTimeStep())

sensor = Sensor(robot)
sensor.enable(timestep)

motor = Actuator(robot)
motor.arming(0.0)

keyboard = Keyboard()
keyboard.enable(timestep)

controller = Controller(
    roll_param=roll_param,
    pitch_param=pitch_param,
    yaw_param=yaw_param,
    z_param=z_param_takeoff,
)

while robot.step(timestep) != -1:
    imu = sensor.read_imu()
    gyro = sensor.read_gyro()
    gps = sensor.read_gps()
    head = sensor.read_compass()
    image = sensor.read_camera()

    # print("roll={: .2f}|pitch={: .2f}|yaw={: .2f}".format(imu[0], imu[1], imu[2]))
    # print("roll_accel={: .2f} | pitch_accel={: .2f} | yaw_accel={: .2f}".format(gyro[0], gyro[1], gyro[2]))
    # print("x_pos={: .2f}|y_pos={: .2f}|z_pos={: .2f}".format(gps[0], gps[1], gps[2]))
    # print("heading={: .2f}".format(head))
    # print("x_tar={: .2f}|y_tar={: .2f}|z_tar={: .2f}|yaw_tar={: .2f}".format(x_target, y_target, z_target, yaw_target))

    key = keyboard.getKey()
    while key > 0:
        if key == ord("T") and status_takeoff == False:
            status_takeoff = True
            status_landing = False
            motor.arming(arming_speed=10.0)
            motor.gimbal_down(pitch_angle=1.6)
            z_target = 3.0
            print("Arming and Take Off")
            sleep(0.25)
            break
        if key == Keyboard.UP:
            z_target += 0.01
            print("z_target=", z_target)
            break
        if key == Keyboard.DOWN:
            z_target -= 0.01
            print("z_target=", z_target)
            break
        if key == Keyboard.RIGHT:
            yaw_target += 0.001
            if yaw_target >= 3.14:
                yaw_target = 3.14
            elif yaw_target <= -3.14:
                yaw_target = -3.14
            print("yaw_target=", yaw_target)
            break
        if key == Keyboard.LEFT:
            yaw_target -= 0.001
            if yaw_target >= 3.14:
                yaw_target = 3.14
            elif yaw_target <= -3.14:
                yaw_target = -3.14
            print("yaw_target=", yaw_target)
            break
        if key == ord("W"):
            x_target -= 0.1
            print("x_target=", x_target)
            break
        if key == ord("S"):
            x_target += 0.1
            print("x_target=", x_target)
            break
        if key == ord("A"):
            y_target += 0.1
            print("y_target=", y_target)
            break
        if key == ord("D"):
            y_target -= 0.1
            print("y_target=", y_target)
            break
        if key == Keyboard.HOME:
            print(" Go Home")
            x_target = 0.0
            y_target = 0.0
            z_target = 10.0
            yaw_target = 0.0
            sleep(0.25)
            break
        if key == ord("L") and status_landing == False:
            status_landing = True
            status_takeoff = False
            z_target = 0.0
            print("Landing")
            sleep(0.25)
            break

    target = [x_target, y_target, z_target, yaw_target]

    x_error, y_error, z_error, yaw_error = controller.error_calculation(
        target=target, gps=gps, marker=marker_pos, head=head, status=status_aruco
    )

    x_int_error = x_int_error + x_error
    y_int_error = y_int_error + y_error
    z_int_error = z_int_error + z_error
    yaw_int_error = yaw_int_error + yaw_error

    x_dif_error = x_error - x_bef_error
    y_dif_error = y_error - y_bef_error
    z_dif_error = z_error - z_bef_error
    yaw_dif_error = yaw_error - yaw_bef_error

    x_bef_error = x_error
    y_bef_error = y_error
    z_bef_error = z_error
    yaw_bef_error = yaw_error

    x_err = [x_error, x_int_error, x_dif_error]
    y_err = [y_error, y_int_error, y_dif_error]
    z_err = [z_error, z_int_error, z_dif_error]
    yaw_err = [yaw_error, yaw_int_error, yaw_dif_error]

    action = controller.calculate(imu=imu, gyro=gyro, error=[x_err, y_err, z_err, yaw_err], head=head)
    gimbal_cal = controller.gimbal_control(gyro=gyro, pitch_angle=1.6)

    if (status_landing == True and z_error < 0.1) or (status_takeoff == False and status_landing == False):
        motor.arming(arming_speed=0.0)

    print(
        "act_0={: .2f}|act_1={: .2f}|act_2={: .2f}|act_3={: .2f}|act_4={: .2f}".format(
            action[0], action[1], action[2], action[3], action[4]
        )
    )

    motor_fl = action[0] + action[1] - action[2] - action[3] + action[4]
    motor_fr = action[0] + action[1] + action[2] - action[3] - action[4]
    motor_rl = action[0] + action[1] - action[2] + action[3] - action[4]
    motor_rr = action[0] + action[1] + action[2] + action[3] + action[4]

    motor.motor_speed(motor_fl=motor_fl, motor_fr=motor_fr, motor_rl=motor_rl, motor_rr=motor_rr)
    motor.gimbal_down(gimbal_cal[0], gimbal_cal[1], gimbal_cal[2])

    cv2.imshow("camera", image)
    cv2.waitKey(1)

cv2.destroyAllWindows()
