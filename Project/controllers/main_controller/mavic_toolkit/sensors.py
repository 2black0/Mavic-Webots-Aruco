"""
Drone Sensor Suite Module
========================

This module provides a unified interface to access all sensor data from the
Mavic drone in the Webots simulator. It abstracts the details of working with
individual sensors and provides convenient methods to retrieve processed data.
"""

import numpy as np
import cv2
from controller import Robot
from .config import ControlParams

class SensorSuite:
    """
    Unified access to all drone sensor devices with data processing.
    
    Provides a clean interface to initialize and read from IMU, gyroscope,
    GPS, compass, and camera sensors. Handles unit conversions and data
    formatting to provide consistent outputs for the control system.
    
    Attributes:
        robot (Robot): Webots Robot instance for device access.
        timestep (int): Control timestep in milliseconds.
        config (ControlParams): Control parameters used for configuration.
        imu (InertialUnit): Provides roll, pitch, yaw orientation data.
        gyro (Gyro): Provides angular velocity measurements.
        gps (GPS): Provides absolute position in the world.
        compass (Compass): Provides heading information.
        camera (Camera): Provides visual data for ArUco detection.
    """

    def __init__(self, robot: Robot):
        """
        Initialize all sensor references from the robot.
        
        Parameters:
            robot (Robot): Webots Robot instance with sensor devices.
        """
        self.robot    = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.config   = ControlParams()

        # Get device references from Webots robot
        self.imu     = robot.getDevice("inertial unit")
        self.gyro    = robot.getDevice("gyro")
        self.gps     = robot.getDevice("gps")
        self.compass = robot.getDevice("compass")
        self.camera  = robot.getDevice("camera")

    # ------------------------------------------------------------------ #
    def enable_all(self):
        """
        Enable all sensors at the robot's timestep rate.
        
        This method must be called before attempting to read any sensor data.
        All sensors will update at the same rate as the control loop.
        """
        for dev in (self.imu, self.gyro, self.gps, self.compass, self.camera):
            dev.enable(self.timestep)

    # ------------------------------------------------------------------ #
    #  Read‑outs
    # ------------------------------------------------------------------ #
    def get_orientation(self):
        """
        Get the drone's orientation in degrees.
        
        Returns:
            ndarray: Roll, pitch, yaw angles in degrees (matches legacy conventions).
                     Positive roll = right wing down
                     Positive pitch = nose up
                     Positive yaw = counterclockwise rotation
        """
        return np.degrees(self.imu.getRollPitchYaw())

    def get_position(self):
        """
        Get the drone's absolute position in the world.
        
        Returns:
            ndarray: X, Y, Z position in meters in the Webots coordinate system.
                     X = forward/backward
                     Y = left/right
                     Z = up/down (altitude)
        """
        return np.array(self.gps.getValues())

    def get_camera_frame(self):
        """
        Get a BGR color image from the drone camera.
        
        Processes the raw BGRA data from Webots to a standard BGR format
        used by OpenCV and creates a writable copy for in-place modifications.
        
        Returns:
            ndarray: BGR color image as uint8 array of shape (height, width, 3).
                     This is a writable copy that can be modified in-place.
        
        Raises:
            RuntimeError: If the camera has not been enabled before reading.
        """
        if self.camera.getSamplingPeriod() == 0:
            raise RuntimeError("Camera not enabled!")

        # Get raw image data and reshape to 4-channel BGRA format
        w, h = self.camera.getWidth(), self.camera.getHeight()
        bgra = np.frombuffer(self.camera.getImage(), np.uint8).reshape((h, w, 4))
        
        # Convert to 3-channel BGR (standard OpenCV format) and make a writable copy
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR).copy()