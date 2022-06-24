from controller import (
    Robot,
    Motor,
    Camera,
    InertialUnit,
    Gyro,
    Compass,
    GPS,
    LED,
    Keyboard,
)

robot = Robot()

frontLeftMotor = robot.getDevice("front left propeller")
frontRightMotor = robot.getDevice("front right propeller")
backLeftMotor = robot.getDevice("rear left propeller")
backRightMotor = robot.getDevice("rear right propeller")

frontLeftMotor.setPosition(float("+inf"))
frontRightMotor.setPosition(float("+inf"))
backLeftMotor.setPosition(float("+inf"))
backRightMotor.setPosition(float("+inf"))

takeoffSignal = 30
frontLeftMotor.setVelocity(takeoffSignal)
frontRightMotor.setVelocity(-takeoffSignal)
backLeftMotor.setVelocity(-takeoffSignal)
backRightMotor.setVelocity(takeoffSignal)

timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(timestep)

front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")

gps = robot.getDevice("gps")
gps.enable(timestep)

imu = robot.getDevice("inertial unit")
imu.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

z_coord = []
x_coord = []
y_coord = []

while robot.step(timestep) != -1:
    led_state = int(robot.getTime()) % 2
    front_left_led.set(led_state)
    front_right_led.set(led_state)

    roll = imu.getRollPitchYaw()[0]
    pitch = imu.getRollPitchYaw()[1]
    yaw = compass.getValues()[1]

    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]

    zGPS = gps.getValues()[2]
    z_coord.append(zGPS)
    xGPS = gps.getValues()[0]
    x_coord.append(xGPS)
    yGPS = gps.getValues()[1]
    y_coord.append(yGPS)

    print(
        "R:{:.2f} | P:{:.2f} | Y:{:.2f} | GX:{:.2f} | GY:{:.2f} | GZ:{:.2f} | RA:{:.2f} | PA:{:.2f}".format(
            roll, pitch, yaw, xGPS, yGPS, zGPS, roll_acceleration, pitch_acceleration
        )
    )

    takeoffSignal = 60
    rollSignal = (roll + 1.57) * 10
    pitchSignal = (pitch - 0.07) * 10
    throttleSignal = (zGPS + 2) * 10

    frontLeftMotorSpeed = takeoffSignal - rollSignal - pitchSignal + throttleSignal
    frontRightMotorSpeed = takeoffSignal + rollSignal - pitchSignal + throttleSignal
    backLeftMotorSpeed = takeoffSignal - rollSignal + pitchSignal + throttleSignal
    backRightMotorSpeed = takeoffSignal + rollSignal + pitchSignal + throttleSignal

    frontLeftMotor.setVelocity(frontLeftMotorSpeed)
    frontRightMotor.setVelocity(-frontRightMotorSpeed)
    backLeftMotor.setVelocity(-backLeftMotorSpeed)
    backRightMotor.setVelocity(backRightMotorSpeed)

# roll zero -1.57
# roll kiri -1.67
# roll kanan -1.47

# pitch zero 0.07
# pitch dangak 0.33
# pitch nunduk - 0.19

# height zero 0.00
# naik -0.04
# turun 0.04
