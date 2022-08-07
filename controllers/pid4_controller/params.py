x_param = [2, 0.0001, 5]
y_param = [2, 0.0001, 5]
z_param = [3, 0.08, 10]

roll_param = [0.35, 0, 0.35]
# roll_accel_param = [0.55, 0, 0.5]
pitch_param = [0.40, 0, 0.45]
# pitch_accel_param = [0.65, 0, 0.5]
yaw_param = [0.75, 0, 0.25]

vertical_thrust = 68.5

x_target = 0.0
y_target = 0.0
z_target = 0.0
yaw_target = 0.0
roll_target = 0.0
pitch_target = 0.0

y_error = 0.0
x_error = 0.0

status_takeoff = False
status_landing = False
status_gimbal = False
status_aruco = False

roll_gimbal_angle = 0.0
pitch_gimbal_angle = 0.0
yaw_gimbal_angle = 0.0
