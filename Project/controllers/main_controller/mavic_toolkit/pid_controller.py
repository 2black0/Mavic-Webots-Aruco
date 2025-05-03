"""
PID Controller Module
====================

This module provides PID (Proportional-Integral-Derivative) controllers for
all the drone's control axes. It wraps the simple-pid package with proper
configuration and convenient helper methods for the Mavic drone control.
"""

from simple_pid import PID
from .config import PIDParams, ControlParams

class PIDController(dict):
    """
    Dictionary-based container for multiple PID controllers.
    
    Subclasses dict to provide access to individual PID controllers via
    dictionary syntax (e.g., pid_ctl['roll'](value)) while adding helper
    methods for common operations like setting multiple setpoints at once.
    
    Each controller is pre-configured with the appropriate PID gains and
    output limits from the configuration parameters.
    
    Attributes:
        params (PIDParams): Reference to the PID parameters used for initialization
        
    Example:
        # Create controller and set setpoints
        pid_ctl = PIDController()
        pid_ctl.setpoints(yaw=0.0, z=3.0)
        
        # Use individual controllers
        roll_correction = pid_ctl['roll'](current_roll)
        pitch_correction = pid_ctl['pitch'](current_pitch)
    """

    def __init__(self):
        """
        Initialize PID controllers for all axes with configured parameters.
        
        Creates separate PID controllers for x, y, z position control and
        roll, pitch, yaw orientation control, each with their own tuned
        parameters and appropriate output limits.
        """
        super().__init__()
        self.params = PIDParams()
        cfg         = ControlParams()

        # Build PID objects for each axis
        super().update({
            # Position controllers
            "x"    : PID(*self.params.x),         # Forward/backward position
            "y"    : PID(*self.params.y),         # Left/right position
            "z"    : PID(*self.params.z,          # Altitude control
                       output_limits=cfg.control_limits["alti"]),
                       
            # Attitude controllers
            "roll" : PID(*self.params.roll),      # Roll angle control
            "pitch": PID(*self.params.pitch),     # Pitch angle control
            "yaw"  : PID(*self.params.yaw,        # Yaw angle control
                       output_limits=cfg.control_limits["yaw"]),
        })

    # ------------------------------------------------------------------ #
    # Convenience helpers
    # ------------------------------------------------------------------ #
    def setpoints(self, **kwargs):
        """
        Set multiple PID controller setpoints at once.
        
        This is a convenience method to update multiple setpoints with a
        single function call instead of setting each one individually.
        
        Parameters:
            **kwargs: Key-value pairs where keys are axis names ('x', 'y', 'z', 
                     'roll', 'pitch', 'yaw') and values are the desired setpoints.
        
        Example:
            pid_controller.setpoints(yaw=0.0, z=3.0, x=1.5)
        """
        for axis, sp in kwargs.items():
            self[axis].setpoint = float(sp)