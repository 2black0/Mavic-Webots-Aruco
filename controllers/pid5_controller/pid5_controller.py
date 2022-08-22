from tracemalloc import start
from controller import Robot, Keyboard
from simple_pid import PID
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
from csv_logger import CsvLogger

filename = "logger.csv"
header = [
    "time",
    "counter",
    "roll",
    "pitch",
    "yaw",
    "roll_accel",
    "pitch_accel",
    "yaw_accel",
    "xpos",
    "ypos",
    "altitude",
    "roll_error",
    "pitch_error",
    "roll_input",
    "pitch_input",
    "yaw_input",
    "vertical_input",
    "front_left_motor_input",
    "front_right_motor_input",
    "rear_left_motor_input",
    "rear_right_motor_input",
    "Speed_X",
    "Speed_Y",
    "Speed_Z",
    "status_takeoff",
    "status_home",
    "status_aruco",
    "status_landing",
]
csvlogger = CsvLogger(filename=filename, header=header)


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic(Robot):
    K_VERTICAL_THRUST = 68.5
    X_PID = [2, 2, 4]
    Y_PID = [1.5, 2, 3]
    ALTI_PID = [4, 0.05, 10]
    ROLL_PID = [45, 1, 7]
    PITCH_PID = [35, 1, 7]
    YAW_PID = [0.5, 0.0075, 3]

    counter = 0

    x_target = 0.0
    y_target = 0.0
    yaw_target = 0.0
    alti_target = 0.0

    x_target_aruco = 0.0
    y_target_aruco = 0.0

    roll_angle_gimbal = 0.0
    pitch_angle_gimbal = 0.0
    yaw_angle_gimbal = 0.0

    status_takeoff = False
    status_landing = False
    status_gimbal = False
    status_home = False
    status_aruco = False

    status_home_A = False
    status_home_B = False
    status_home_C = False

    yawPID = PID(float(YAW_PID[0]), float(YAW_PID[1]), float(YAW_PID[2]), setpoint=float(yaw_target))
    altiPID = PID(float(ALTI_PID[0]), float(ALTI_PID[1]), float(ALTI_PID[2]), setpoint=float(alti_target))

    yawPID.output_limits = (-0.5, 0.5)
    altiPID.output_limits = (-1.5, 1.5)

    def __init__(self):
        Robot.__init__(self)
        self.timeStep = int(self.getBasicTimeStep())

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)
        self.water_to_drop = 0

        self.camera = self.getDevice("camera")
        self.camera.enable(self.timeStep)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.timeStep)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.timeStep)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(1)

        self.camera_roll = self.getDevice("camera roll")
        self.camera_pitch = self.getDevice("camera pitch")
        self.camera_yaw = self.getDevice("camera yaw")
        # gimbals = [self.camera_roll, self.camera_pitch, self.camera_yaw]
        # for gimbal in gimbals:
        #    gimbal.setPosition(0.0)

    def convert_to_attitude(self, x_error, y_error, yaw):
        self.c, self.s = np.cos(yaw), np.sin(yaw)
        self.R = np.array(((self.c, -self.s), (self.s, self.c)))
        self.converted = np.matmul([x_error, y_error], self.R)
        return self.converted

    def read_camera(self):
        self.camera_height = self.camera.getHeight()
        self.camera_width = self.camera.getWidth()
        self.image = self.camera.getImage()
        self.image = np.frombuffer(self.image, np.uint8).reshape((self.camera_height, self.camera_width, 4))
        return self.image

    def find_aruco(self, image):
        self.image = image
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.corner, self.id, self.reject = aruco.detectMarkers(self.gray, self.aruco_dict, parameters=self.parameters)
        return self.corner, self.id, self.reject

    def run(self, show=False, log=False):
        counter = 0
        while self.step(self.timeStep) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            roll_accel, pitch_accel, yaw_accel = self.gyro.getValues()
            xpos, ypos, altitude = self.gps.getValues()
            image = self.read_camera()

            key = self.keyboard.getKey()

            # Command
            ## takeoff
            if key == ord("T"):
                self.status_takeoff = True
                self.status_gimbal = True
                self.alti_target = 3.0
                self.pitch_angle_gimbal = 1.6
                print("Take Off")
                sleep(0.15)
            ## landing
            elif key == Keyboard.END:
                self.status_landing = True
                self.alti_target = 0.1
                print("Landing")
                sleep(0.15)
            ## gimbal
            elif key == ord("G"):
                self.status_gimbal = not self.status_gimbal
                if self.status_gimbal == True:
                    self.roll_angle_gimbal = 0.0
                    self.pitch_angle_gimbal = 1.6
                    self.yaw_angle_gimbal = 0.0
                print("Gimbal Stabilize", self.status_gimbal)
                sleep(0.15)
            elif key == ord("I"):
                self.pitch_angle_gimbal += 0.005
                if self.pitch_angle_gimbal >= 1.7:
                    self.pitch_angle_gimbal = 1.7
                print("Pitch Gimbal Angle:", self.pitch_angle_gimbal)
            elif key == ord("K"):
                self.pitch_angle_gimbal -= 0.005
                if self.pitch_angle_gimbal <= -0.5:
                    self.pitch_angle_gimbal = -0.5
                print("Pitch Gimbal Angle:", self.pitch_angle_gimbal)
            elif key == ord("J"):
                self.roll_angle_gimbal += 0.005
                if self.roll_angle_gimbal >= 0.5:
                    self.roll_angle_gimbal = 0.5
                print("Roll Gimbal Angle:", self.roll_angle_gimbal)
            elif key == ord("L"):
                self.roll_angle_gimbal -= 0.005
                if self.roll_angle_gimbal <= -0.5:
                    self.roll_angle_gimbal = -0.5
                print("Roll Gimbal Angle:", self.roll_angle_gimbal)
            elif key == ord("U"):
                self.yaw_angle_gimbal += 0.005
                if self.yaw_angle_gimbal >= 1.7:
                    self.yaw_angle_gimbal = 1.7
                print("Yaw Gimbal Angle:", self.yaw_angle_gimbal)
            elif key == ord("O"):
                self.yaw_angle_gimbal -= 0.005
                if self.yaw_angle_gimbal <= -1.7:
                    self.yaw_angle_gimbal = -1.7
                print("Yaw Gimbal Angle:", self.yaw_angle_gimbal)
            # moving
            elif key == ord("W"):
                self.x_target += 0.1
                print("target x:{: .2f}[m]".format(self.x_target))
            elif key == ord("S"):
                self.x_target -= 0.1
                print("target x:{: .2f}[m]".format(self.x_target))
            elif key == ord("A"):
                self.y_target += 0.1
                print("target y:{: .2f}[m]".format(self.y_target))
            elif key == ord("D"):
                self.y_target -= 0.1
                print("target y:{: .2f}[m]".format(self.y_target))
            elif key == Keyboard.LEFT:
                self.yaw_target += 0.05
                print("target yaw:{: .2f}[rad]".format(self.yaw_target))
            elif key == Keyboard.RIGHT:
                self.yaw_target -= 0.05
                print("target yaw:{: .2f}[rad]".format(self.yaw_target))
            elif key == Keyboard.UP:
                self.alti_target += 0.05
                print("target altitude:{: .2f}[m]".format(self.alti_target))
            elif key == Keyboard.DOWN:
                self.alti_target -= 0.05
                print("target altitude:{: .2f}[m]".format(self.alti_target))
            ## home
            elif key == Keyboard.HOME:
                self.status_home = not self.status_home
                if self.status_home == True:
                    self.x_target = 0.0
                    self.y_target = 0.0
                    self.yaw_target = 0.0
                    self.alti_target = 10.0
                print("Status Home:", self.status_home)
                sleep(0.15)
            ## aruco
            elif key == ord("M"):
                self.status_aruco = not self.status_aruco
                print("Status Aruco:", self.status_aruco)
                sleep(0.15)
            ## rth
            elif key == Keyboard.PAGEDOWN:
                self.status_home_A = not self.status_home_A
                self.status_aruco = not self.status_aruco
                if self.status_home_A == True:
                    self.x_target = 0.0
                    self.y_target = 0.0
                    self.yaw_target = 0.0
                    self.alti_target = 20.0
                    # self.status_home_A = True
                print("Return To Home:", self.status_home_A)
                sleep(0.15)

            """
            if self.status_home_A == True:
                error_X = xpos - self.x_target
                error_Y = ypos - self.y_target
                error_alti = altitude - self.alti_target
                print(error_alti)
                if (
                    (error_X < 0.25 and error_X > -0.25)
                    and (error_Y < 0.25 and error_Y > -0.25)
                    and (error_alti < 0.25 and error_alti > -0.25)
                ):
                    self.status_home_A = False
                    print(self.status_home_A)
            # elif self.status_home_A == True and self.status_takeoff == True:
            #    self.status_aruco = True
            """

            cam_height = int(self.camera.getHeight())  # 240
            cam_width = int(self.camera.getWidth())  # 400
            # print(cam_height, cam_width)

            if self.status_aruco == True:
                corner, id, _ = self.find_aruco(image=image)
                if id is not None:
                    # counter += 1
                    # print(counter)
                    image = cv2.line(
                        image, (int(cam_width / 2), cam_height), (int(cam_width / 2), 0), (255, 255, 0), 1
                    )
                    image = cv2.line(
                        image, (0, int(cam_height / 2)), (cam_width, int(cam_height / 2)), (255, 255, 0), 1
                    )

                    center_x = corner[0][0][1][0] + ((corner[0][0][0][0] - corner[0][0][1][0]) / 2)
                    center_y = corner[0][0][2][1] + ((corner[0][0][0][1] - corner[0][0][2][1]) / 2)
                    # image = cv2.circle(image, (int(center_x), int(center_y)), 2, (0, 0, 255), 3)
                    start_point = (int(corner[0][0][1][0]), int(corner[0][0][0][1]))
                    end_point = (int(corner[0][0][0][0]), int(corner[0][0][2][1]))
                    # shapes = np.zeros_like(image, np.uint8)
                    shapes = image.copy()
                    cv2.rectangle(shapes, start_point, end_point, (0, 255, 0), -1)
                    alpha = 0.4
                    image = cv2.addWeighted(shapes, alpha, image, 1 - alpha, 0)
                    self.x_target_aruco = -4 * ((center_y - (cam_height / 2)) / cam_height)
                    self.y_target_aruco = 4 * ((center_x - (cam_width / 2)) / cam_width)
                    roll_error = clamp(-self.y_target_aruco + 0.06, -1.5, 1.5)
                    pitch_error = clamp(self.x_target_aruco - 0.13, -1.5, 1.5)
                    # print("xe={: .2f}|ye={: .2f}".format(self.x_target_aruco, self.y_target_aruco))
                    error_alti = altitude - self.alti_target
                    if (
                        (self.x_target_aruco < 0.1 and self.x_target_aruco > -0.1)
                        and (self.y_target_aruco < 0.1 and self.y_target_aruco > -0.1)
                        and (error_alti < 0.5 and error_alti > -0.5)
                        and self.status_landing == False
                    ):
                        self.status_landing = True
                        self.alti_target = 0.0
                        print("Landing")
                else:
                    roll_error = clamp(-ypos + 0.06 + self.y_target, -1.5, 1.5)
                    pitch_error = clamp(-xpos - 0.13 + self.x_target, -1.5, 1.5)
            else:
                roll_error = clamp(-ypos + 0.06 + self.y_target, -1.5, 1.5)
                pitch_error = clamp(-xpos - 0.13 + self.x_target, -1.5, 1.5)

            self.yawPID.setpoint = self.yaw_target
            self.altiPID.setpoint = self.alti_target

            roll_input = (self.ROLL_PID[0] * clamp(roll, -1, 1)) + (self.ROLL_PID[2] * roll_accel) + roll_error
            pitch_input = (self.PITCH_PID[0] * clamp(pitch, -1, 1)) + (self.PITCH_PID[2] * pitch_accel) - pitch_error
            yaw_input = self.yawPID(yaw)
            vertical_input = self.altiPID(altitude)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            if self.status_takeoff == False or (self.status_landing == True and altitude <= 0.1):
                self.status_takeoff = False
                self.status_landing = False
                self.status_gimbal = False
                self.status_home = False
                front_left_motor_input = 0.0
                front_right_motor_input = 0.0
                rear_left_motor_input = 0.0
                rear_right_motor_input = 0.0

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)

            if self.status_gimbal == True:
                roll_gimbal = clamp((-0.001 * roll_accel + self.roll_angle_gimbal), -0.5, 0.5)
                pitch_gimbal = clamp(((-0.001 * pitch_accel) + self.pitch_angle_gimbal), -0.5, 1.7)
                yaw_gimbal = clamp((-0.001 * yaw_accel + self.yaw_angle_gimbal), -1.7, 1.7)
                self.camera_roll.setPosition(roll_gimbal)
                self.camera_pitch.setPosition(pitch_gimbal)
                self.camera_yaw.setPosition(yaw_gimbal)

            speed = self.gps.getSpeedVector()
            # print("SX={:+.2f}|SY={:+.2f}|SZ={:+.2f}".format(speed[0], speed[1], speed[2]))

            logs = [
                roll,
                pitch,
                yaw,
                roll_accel,
                pitch_accel,
                yaw_accel,
                xpos,
                ypos,
                altitude,
                roll_error,
                pitch_error,
                roll_input,
                pitch_input,
                yaw_input,
                vertical_input,
                front_left_motor_input,
                front_right_motor_input,
                rear_left_motor_input,
                rear_right_motor_input,
                speed[0],
                speed[1],
                speed[2],
                self.status_takeoff,
                self.status_home,
                self.status_aruco,
                self.status_landing,
            ]

            dlogs = []
            for i in logs:
                dlogs.append(float("{:.2f}".format(i)))

            debug_mode = show
            if debug_mode == True:
                print(
                    "r={:+.2f}|p={:+.2f}|y={:+.2f}|ra={:+.2f}|pa={:+.2f}|ya={:+.2f}|x={:+.2f}|y={:+.2f}|z={:+.2f}|re={:+.2f}|pe={:+.2f}|ri={:+.2f}|pi={:+.2f}|yi={:+.2f}|vi={:+.2f}|fl={:+.2f}|fr={:+.2f}|rl={:+.2f}|rr={:+.2f}|sx={:+.2f}|sy={:+.2f}|sz={:+.2f}|st={}|sh={}|sa={}|sl={}".format(
                        dlogs[0],
                        dlogs[1],
                        dlogs[2],
                        dlogs[3],
                        dlogs[4],
                        dlogs[5],
                        dlogs[6],
                        dlogs[7],
                        dlogs[8],
                        dlogs[9],
                        dlogs[10],
                        dlogs[11],
                        dlogs[12],
                        dlogs[13],
                        dlogs[14],
                        dlogs[15],
                        dlogs[16],
                        dlogs[17],
                        dlogs[18],
                        dlogs[19],
                        dlogs[20],
                        dlogs[21],
                        int(dlogs[22]),
                        int(dlogs[23]),
                        int(dlogs[24]),
                        int(dlogs[25]),
                    )
                )

            cv2.imshow("Camera", image)
            cv2.waitKey(1)

            log_mode = log
            if log_mode == True:
                dlogs.insert(0, counter)
                csvlogger.critical(dlogs)

            counter += 1
        cv2.destroyAllWindows()


robot = Mavic()
robot.run(show=False, log=False)
