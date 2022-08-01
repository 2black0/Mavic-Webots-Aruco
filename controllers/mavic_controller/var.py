# roll_param = [50, 0, 0]
roll_param = [50, 0, 10]
# pitch_param = [30, 0, 0]
pitch_param = [35, 0, 10]
yaw_param = [0.05, 0, 0]
z_param_takeoff = [0.75, 0.0003, 10]
z_param_landing = [0.1, 0.0003, 10]

x_target = 0.0
y_target = 0.0
z_target = 0.0
yaw_target = 0.0

marker_pos = [0, 0, 0, 0]
id = None

status_aruco = False
status_takeoff = False
status_landing = False
status_button = False
status_gimbal = False

x_int_error = 0
y_int_error = 0
z_int_error = 0
yaw_int_error = 0

x_bef_error = 0
y_bef_error = 0
z_bef_error = 0
yaw_bef_error = 0

x_dif_error = 0
y_dif_error = 0
z_dif_error = 0
yaw_dif_error = 0
