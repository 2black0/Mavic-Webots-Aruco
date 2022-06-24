from controller import Robot, Motor, Camera, InertialUnit, Gyro, Compass, GPS, LED, Keyboard
import cv2
import numpy
from simple_pid import PID
import csv
import matplotlib.pyplot as plt
import struct
import math

params = dict()
with open("../params.csv", "r") as f:
	lines = csv.reader(f)
	for line in lines:
		params[line[0]] = line[1]

TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = numpy.pi;

X_init = -2.0
Z_init = -3.0
Y_init = 0.0

X_fin = 1.0
Z_fin = 2.0
Y_fin = 0.0

h = (X_init + X_fin)/2.0
k = (Z_init + Z_fin)/2.0

r = (math.sqrt((X_fin-X_init)**2 + (Z_fin-Z_init)**2))/2.0
ts = 0.032
n = 800
t1 = n*ts
w = M_PI/n
theta_i = M_PI
theta_f = 0.0

print(r)

def X_val(n):
    targetX = h + ((h-X_init)*math.cos(theta_i - w*n))
    return targetX
    
def Y_val(n):
    targetY = r*math.sin(theta_i - w*n)
    print('hello')
    return targetY
    
def Z_val(n):
    targetZ = k + ((k-Z_init)*math.cos(theta_i - w*n))
    return targetZ
    

robot = Robot()

frontLeftMotor = robot.getDevice('front left propeller')
frontRightMotor = robot.getDevice('front right propeller')
backLeftMotor = robot.getDevice('rear left propeller')
backRightMotor = robot.getDevice('rear right propeller')

timestep = int(robot.getBasicTimeStep())
frontLeftMotor.setPosition(float('+inf'))
frontRightMotor.setPosition(float('+inf'))
backLeftMotor.setPosition(float('+inf'))
backRightMotor.setPosition(float('+inf'))

frontLeftMotor.setVelocity(TAKEOFF_THRESHOLD_VELOCITY)
frontRightMotor.setVelocity(TAKEOFF_THRESHOLD_VELOCITY)
backLeftMotor.setVelocity(TAKEOFF_THRESHOLD_VELOCITY)
backRightMotor.setVelocity(TAKEOFF_THRESHOLD_VELOCITY)


camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)
imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)


pitchPID = PID(float(params["pitch_Kp"]), float(params["pitch_Ki"]), float(params["pitch_Kd"]), setpoint=0.0)
rollPID = PID(float(params["roll_Kp"]), float(params["roll_Ki"]), float(params["roll_Kd"]), setpoint=0.0)
throttlePID = PID(float(params["throttle_Kp"]), float(params["throttle_Ki"]), float(params["throttle_Kd"]), setpoint=0.0)
yawPID = PID(float(params["yaw_Kp"]), float(params["yaw_Ki"]), float(params["yaw_Kd"]), setpoint=float(params["yaw_setpoint"]))
    	
altitude_attained = True
    
time=[]
z_coord=[]
x_coord=[]
y_coord=[]
x_cal=[]
y_cal=[]
z_cal=[]
i = 0
    
while (robot.step(timestep) != -1):
    
    led_state = int(robot.getTime()) % 2
    front_left_led.set(led_state)
    front_right_led.set(int(not(led_state)))
    
    if Y_init == 0 and Y_fin == 0:
        throttlePID.setpoint = Y_val(i)	
        y_cal.append(throttlePID.setpoint)
    else:
        throttlePID.setpoint = Y_fin
        print('bye')
        
    t=robot.getTime()
    
    time.append(t)
    print(t)
    
    roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
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
    
    print(i)
   
    
    vertical_input = throttlePID(yGPS)
    yaw_input = yawPID(yaw)
   
    if Y_init == 0 and Y_fin == 0:
        rollPID.setpoint = -(Z_val(i))
        pitchPID.setpoint = (X_val(i))
        x_cal.append(pitchPID.setpoint)
        z_cal.append(rollPID.setpoint)
    else:
        rollPID.setpoint = -Z_fin
        pitchPID.setpoint = X_fin    
    	
    roll_input = float(params["k_roll_p"]) * roll + roll_acceleration + rollPID(-zGPS)
    pitch_input = float(params["k_pitch_p"]) * pitch - pitch_acceleration + pitchPID(xGPS)
    
    
    i = i+1
    
    front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input + pitch_input + yaw_input
    	
    if i ==800:
       break
    
    if not(numpy.isnan(front_left_motor_input)):
                frontLeftMotor.setVelocity(front_left_motor_input)
                frontRightMotor.setVelocity(-front_right_motor_input)
                backLeftMotor.setVelocity(-rear_left_motor_input)
                backRightMotor.setVelocity(rear_right_motor_input)   

plt.subplot(2,2,1)   
plt.plot(x_coord, y_coord)
plt.xlabel("X axis")
plt.ylabel("altitude")
plt.subplot(2,2,2)
plt.plot(z_coord, y_coord)
plt.xlabel("Z axis")
plt.ylabel("altitude")
plt.subplot(2,2,3)   
plt.plot(x_cal, y_cal)
plt.xlabel("X calculated")
plt.ylabel("altitude")
plt.subplot(2,2,4)
plt.plot(z_cal, y_cal)
plt.xlabel("Z calculated")
plt.ylabel("altitude")
plt.show()		
