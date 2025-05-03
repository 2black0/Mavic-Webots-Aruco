"""
Mavic Toolkit Package
=====================

A comprehensive toolkit for controlling a Mavic drone in the Webots simulator
with ArUco marker-based precision landing capabilities.

This package organizes the drone's controller functionality into modular components:
  - sensors: Abstractions for cameras, GPS, IMU, and other sensors
  - actuators: Motor and gimbal control systems
  - markers: ArUco marker detection and processing
  - pid_controller: PID control loops for stable flight
  - config: Parameter definitions and state management
  - command: User input handling via keyboard

Each module is designed to be reusable across different drone control applications
while maintaining clean separation of concerns.
"""

from .sensors import SensorSuite
from .actuators import FlightController, GimbalController
from .markers  import ArucoDetector
from .pid_controller import PIDController
from .config import ControlParams, PIDParams, State
from .command import KeyboardHandler

__all__ = [
    "SensorSuite",
    "FlightController",
    "GimbalController",
    "ArucoDetector",
    "PIDController",
    "ControlParams",
    "PIDParams",
    "State",
    "KeyboardHandler",
]