"""
Configuration Parameters Module
==============================

This module defines the configuration parameters, PID controller values, and 
state tracking for the Mavic drone controller. It uses dataclasses for clean,
typed attribute access and initialization.
"""

from dataclasses import dataclass, field

# --------------------------------------------------------------------------- #
# Gains identical to *aruco_controller*
# --------------------------------------------------------------------------- #
@dataclass
class PIDParams:
    """
    PID controller parameters for all control axes.
    
    Each axis contains [Kp, Ki, Kd] coefficients in a list.
    These values are used to initialize the PID controllers.
    
    Attributes:
        x (list): Position control along X-axis [Kp, Ki, Kd].
        y (list): Position control along Y-axis [Kp, Ki, Kd].
        z (list): Altitude control [Kp, Ki, Kd].
        roll (list): Roll rate control [Kp, Ki, Kd].
        pitch (list): Pitch rate control [Kp, Ki, Kd].
        yaw (list): Yaw rate control [Kp, Ki, Kd].
    """
    x:    list = field(default_factory=lambda: [2, 0.0001, 5])
    y:    list = field(default_factory=lambda: [2, 0.0001, 5])
    z:    list = field(default_factory=lambda: [4, 0.05, 10])      # ALTI_PID
    roll: list = field(default_factory=lambda: [45, 1, 7])         # ROLL_PID
    pitch:list = field(default_factory=lambda: [35, 1, 7])         # PITCH_PID
    yaw:  list = field(default_factory=lambda: [0.5, 0.0075, 3])   # YAW_PID

@dataclass
class ControlParams:
    """
    General control parameters and limits for the drone.
    
    Contains parameters that affect the drone's behavior including
    thrust baseline, camera settings, and control output limits.
    
    Attributes:
        vertical_thrust (float): Baseline thrust to counteract gravity (rad/s).
        camera_resolution (tuple): Width and height of camera in pixels.
        aruco_dict_type (str): ArUco marker dictionary type for detection.
        control_limits (dict): Min/max outputs for control loops.
    """
    vertical_thrust:     float  = 68.5  # Baseline thrust for hovering
    camera_resolution:   tuple  = (400, 240)  # Resolution affects detection range
    aruco_dict_type:     str    = "DICT_6X6_250"  # ArUco dictionary type
    control_limits: dict = field(default_factory=lambda: {
        "yaw":  (-0.5, 0.5),   # Min/max yaw control output
        "alti": (-1.5, 1.5),   # Min/max altitude control output
    })

@dataclass
class State:
    """
    Runtime state container for the drone controller.
    
    Maintains all the mutable state values including position targets,
    gimbal orientation, status flags, and vision-processing errors.
    This object is shared and updated across controller components.
    
    Attributes:
        x_target (float): Target X-position in meters.
        y_target (float): Target Y-position in meters.
        z_target (float): Target Z-position (altitude) in meters.
        yaw_target (float): Target yaw angle in radians.
        
        roll_angle_gimbal (float): Target roll angle for camera gimbal in radians.
        pitch_angle_gimbal (float): Target pitch angle for camera gimbal in radians.
        yaw_angle_gimbal (float): Target yaw angle for camera gimbal in radians.
        
        status_takeoff (bool): Whether motors are enabled and drone is flying.
        status_landing (bool): Whether landing sequence is active.
        status_gimbal (bool): Whether gimbal stabilization is enabled.
        status_home (bool): Whether return-to-home mode is active.
        status_aruco (bool): Whether ArUco tracking is enabled.
        
        status_home_A (bool): Intermediate stage of return-to-home (altitude gain).
        
        x_error_img (float|None): X-axis error from ArUco tracking in meters.
        y_error_img (float|None): Y-axis error from ArUco tracking in meters.
    """
    # Position targets
    x_target: float = 0.0
    y_target: float = 0.0
    z_target: float = 0.0
    yaw_target: float = 0.0

    # Gimbal orientation targets (rad)
    roll_angle_gimbal:  float = 0.0
    pitch_angle_gimbal: float = 0.0
    yaw_angle_gimbal:   float = 0.0

    # Flight status flags
    status_takeoff:  bool = False  # Motors enabled
    status_landing:  bool = False  # Landing sequence active
    status_gimbal:   bool = False  # Gimbal stabilization active
    status_home:     bool = False  # Return-to-home mode active
    status_aruco:    bool = False  # ArUco tracking active

    # Auxiliary (return‑to‑home staged)
    status_home_A: bool = False  # Stage A: gain altitude before returning

    # Vision‑loop temporary errors
    x_error_img: float|None = None  # Forward/backward error from ArUco in meters
    y_error_img: float|None = None  # Left/right error from ArUco in meters