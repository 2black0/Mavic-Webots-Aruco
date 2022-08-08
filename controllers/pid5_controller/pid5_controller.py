from controller import Robot, Keyboard
from simple_pid import PID
from time import sleep


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic(Robot):
    K_VERTICAL_THRUST = 68.5
    X_PID = [2, 2, 10]
    Y_PID = [2, 3, 10]
    ALTI_PID = [2.75, 0.075, 5]
    ROLL_PID = [50, 10, 15]
    PITCH_PID = [40, 10, 15]
    YAW_PID = [0.75, 0, 0.25]

    x_target = 0.0
    y_target = 0.0
    yaw_target = 0.0
    alti_target = 0.0

    roll_angle_gimbal = 0.0
    pitch_angle_gimbal = 0.0
    yaw_angle_gimbal = 0.0

    status_takeoff = False
    status_landing = False
    status_gimbal = False
    status_home = False

    xPID = PID(float(X_PID[0]), float(X_PID[1]), float(X_PID[2]), setpoint=float(x_target))
    yPID = PID(float(Y_PID[0]), float(Y_PID[1]), float(Y_PID[2]), setpoint=float(y_target))
    yawPID = PID(float(YAW_PID[0]), float(YAW_PID[1]), float(YAW_PID[2]), setpoint=float(yaw_target))
    altiPID = PID(float(ALTI_PID[0]), float(ALTI_PID[1]), float(ALTI_PID[2]), setpoint=float(alti_target))

    xPID.output_limits = (-1.5, 1.5)
    yPID.output_limits = (-1.5, 1.5)
    yawPID.output_limits = (-1, 1)
    altiPID.output_limits = (-4, 4)

    def __init__(self):
        Robot.__init__(self)
        self.timeStep = int(self.getBasicTimeStep())

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)
        self.water_to_drop = 0

        # Get and enable devices.
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
        gimbals = [self.camera_roll, self.camera_pitch, self.camera_yaw]
        for gimbal in gimbals:
            gimbal.setPosition(0.0)

    def run(self):
        while self.step(self.timeStep) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            roll_accel, pitch_accel, yaw_accel = self.gyro.getValues()
            xpos, ypos, altitude = self.gps.getValues()

            key = self.keyboard.getKey()

            # Drop the water from the drone
            if key == ord("D"):
                self.water_to_drop += 1
            elif self.water_to_drop > 0:
                self.setCustomData(str(self.water_to_drop))
                self.water_to_drop = 0
            else:
                self.setCustomData(str(0))

            # Movement
            if key == ord("T"):
                self.status_takeoff = True
                self.alti_target = 3.0
                print("Take Off")
                sleep(0.2)
            elif key == ord("L"):
                self.status_landing = True
                self.alti_target = 0.0
                print("Landing")
                sleep(0.2)
            elif key == ord("G"):
                self.status_gimbal = not self.status_gimbal
                if self.status_gimbal == True:
                    self.pitch_angle_gimbal = 1.6
                print("Gimbal Stabilize", self.status_gimbal)
                sleep(0.2)
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
                self.yaw_target += 0.1
                print("target yaw:{: .2f}[rad]".format(self.yaw_target))
            elif key == Keyboard.RIGHT:
                self.yaw_target -= 0.1
                print("target yaw:{: .2f}[rad]".format(self.yaw_target))
            elif key == Keyboard.UP:
                self.alti_target += 0.05
                print("target altitude:{: .2f}[m]".format(self.alti_target))
            elif key == Keyboard.DOWN:
                self.alti_target -= 0.05
                print("target altitude:{: .2f}[m]".format(self.alti_target))
            elif key == Keyboard.HOME:
                self.status_home = not self.status_home
                self.x_target = 0.0
                self.y_target = 0.0
                self.yaw_target = 0.0
                self.alti_target = 10.0
                print("Status Home:", self.status_home)
                sleep(0.2)

            self.xPID.setpoint = self.x_target
            self.yPID.setpoint = self.y_target
            self.yawPID.setpoint = self.yaw_target
            self.altiPID.setpoint = self.alti_target

            roll_error = self.yPID(ypos)
            pitch_error = self.xPID(xpos)

            roll_input = (self.ROLL_PID[0] * clamp(roll, -1, 1)) + (self.ROLL_PID[2] * roll_accel) + roll_error
            pitch_input = (self.PITCH_PID[0] * clamp(pitch, -1, 1)) + (self.PITCH_PID[2] * pitch_accel) - pitch_error
            yaw_input = self.yawPID(yaw)
            vertical_input = self.altiPID(altitude)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            if self.status_takeoff == False or (self.status_landing == True and altitude <= 0.1):
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

            debug_mode = True
            if debug_mode == True:
                print(
                    "r={: .2f}|p={: .2f}|y={: .2f}|ra={: .2f}|pa={: .2f}|ya={: .2f}|x={: .2f}|y={: .2f}|z={: .2f}|re={: .2f}|pe={: .2f}|ri={: .2f}|pi={: .2f}|yi={: .2f}|vi={: .2f}|fl={: .2f}|fr={: .2f}|rl={: .2f}|rr={: .2f}".format(
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
                    )
                )


robot = Mavic()
robot.run()
