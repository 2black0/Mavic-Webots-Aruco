"""
Drone Actuator Control Module
============================

This module provides abstractions for controlling the DJI Mavic's motor and gimbal systems.
It handles the proper sign conventions and initialization requirements for the Webots
simulation environment.
"""

from controller import Robot
from .config import ControlParams

class FlightController:
    """
    Wrapper around four brushless motor devices with DJI‑Mavic sign convention.
    
    Handles motor initialization and provides a clean interface for setting 
    motor speeds with the correct sign conventions for the DJI Mavic drone.
    
    Attributes:
        robot (Robot): Webots Robot instance for device access.
        config (ControlParams): Control parameters for the motors.
        motors (dict): Dictionary of motor devices by position.
    """

    def __init__(self, robot: Robot):
        """
        Initialize the flight controller with motor access.
        
        Parameters:
            robot (Robot): Webots Robot instance to access motor devices.
        """
        self.robot  = robot
        self.config = ControlParams()

        self.motors = {
            "front_left" : robot.getDevice("front left propeller"),
            "front_right": robot.getDevice("front right propeller"),
            "rear_left"  : robot.getDevice("rear left propeller"),
            "rear_right" : robot.getDevice("rear right propeller"),
        }
        self._init_motors()

    def _init_motors(self):
        """
        Initialize all motors with infinite position and arming spin.
        
        Sets each motor to velocity control mode (position = infinity)
        and applies a minimal spin (1.0 rad/s) to simulate motor arming.
        """
        for m in self.motors.values():
            m.setPosition(float("inf"))  # Set to velocity control mode
            m.setVelocity(1.0)           # Arming spin (was 0.0)

    def set_motor_speeds(self, fl: float, fr: float, rl: float, rr: float):
        """
        Set all motor speeds with the correct sign conventions.
        
        In the DJI Mavic convention, motors rotate in alternating directions:
        - Front-left and rear-right: positive = CCW rotation
        - Front-right and rear-left: positive = CW rotation (needs sign inversion)
        
        Parameters:
            fl (float): Front-left motor speed in rad/s.
            fr (float): Front-right motor speed in rad/s.
            rl (float): Rear-left motor speed in rad/s.
            rr (float): Rear-right motor speed in rad/s.
        """
        self.motors["front_left" ].setVelocity(fl)   # CCW rotation
        self.motors["front_right"].setVelocity(-fr)  # CW rotation (inverted)
        self.motors["rear_left"  ].setVelocity(-rl)  # CW rotation (inverted)
        self.motors["rear_right" ].setVelocity(rr)   # CCW rotation


class GimbalController:
    """
    Controller for the three-axis camera gimbal (roll, pitch, yaw).
    
    Provides an interface to set the orientation of the camera gimbal
    for stabilized image capture and targeting.
    
    Attributes:
        devs (dict): Dictionary of gimbal motor devices by axis.
    """

    def __init__(self, robot: Robot):
        """
        Initialize the gimbal controller with access to gimbal motors.
        
        Parameters:
            robot (Robot): Webots Robot instance to access gimbal devices.
        """
        self.devs = {
            "roll" : robot.getDevice("camera roll"),
            "pitch": robot.getDevice("camera pitch"),
            "yaw"  : robot.getDevice("camera yaw"),
        }

    def set_orientation(self, roll: float, pitch: float, yaw: float):
        """
        Set the orientation of all three gimbal axes.
        
        Parameters:
            roll (float): Roll angle in radians.
            pitch (float): Pitch angle in radians.
            yaw (float): Yaw angle in radians.
        """
        self.devs["roll" ].setPosition(roll)
        self.devs["pitch"].setPosition(pitch)
        self.devs["yaw"  ].setPosition(yaw)