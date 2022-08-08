from controller import Robot, Keyboard
from simple_pid import PID


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic(Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    K_VERTICAL_OFFSET = 0.0  # Vertical offset where the robot actually targets to stabilize itself.
    K_ROLL_P = 50.0  # P constant of the roll PID.
    K_PITCH_P = 30.0  # P constant of the pitch PID.

    X_PID = [2, 0, 10]
    Y_PID = [2, 0.01, 5]
    ALTI_PID = [2.75, 0.075, 10]
    ROLL_PID = [30, 10, 15]
    PITCH_PID = [30, 10, 15]
    YAW_PID = [0.75, 0, 0.25]

    x_target = 0.0
    y_target = 0.0
    yaw_target = 0.0
    alti_target = 0.0

    status_takeoff = False
    status_landing = False

    xPID = PID(float(X_PID[0]), float(X_PID[1]), float(X_PID[2]), setpoint=float(x_target))
    yPID = PID(float(Y_PID[0]), float(Y_PID[1]), float(Y_PID[2]), setpoint=float(y_target))
    yawPID = PID(float(YAW_PID[0]), float(YAW_PID[1]), float(YAW_PID[2]), setpoint=float(yaw_target))
    altiPID = PID(float(ALTI_PID[0]), float(ALTI_PID[1]), float(ALTI_PID[2]), setpoint=float(alti_target))

    xPID.output_limits = (-1.5, 1.5)
    yPID.output_limits = (-1.5, 1.5)
    yawPID.output_limits = (-1, 1)
    altiPID.output_limits = (-5, 5)

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

        # Display manual control message.
        print("You can control the drone with your computer keyboard:")
        print("- 'D': drop water")
        print("- 'up': move forward.")
        print("- 'down': move backward.")
        print("- 'right': turn right.")
        print("- 'left': turn left.")
        print("- 'shift + up': increase the target altitude.")
        print("- 'shift + down': decrease the target altitude.")

    def run(self):
        while self.step(self.timeStep) != -1:
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            xpos, ypos, altitude = self.gps.getValues()
            roll_accel, pitch_accel, yaw_accel = self.gyro.getValues()

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
            elif key == ord("L"):
                self.status_landing = True
                self.alti_target = 0.0
                print("Landing")
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


robot = Mavic()
robot.run()
