"""
Keyboard Command Handling Module
===============================

This module provides keyboard input handling for the drone controller, mapping
key presses to specific drone actions and state changes. It updates the shared
State object to maintain separation between input handling and control logic.
"""

from controller import Keyboard
from time import sleep
from .config import State

class KeyboardHandler:
    """
    Handles keyboard input mapping and converts key presses to drone commands.
    
    Updates the State object in-place so the main controller remains decoupled
    from input handling. Provides methods for all common drone operations including
    takeoff, landing, movement, gimbal control, and mode toggles.
    
    Attributes:
        robot (Robot): Webots Robot instance for device access.
        state (State): Shared state object that's updated based on key presses.
        keyboard (Keyboard): Webots keyboard device for input capture.
    """

    def __init__(self, robot, state: State):
        """
        Initialize keyboard handler with robot controller and shared state.
        
        Parameters:
            robot (Robot): Webots Robot instance to access keyboard device.
            state (State): Shared state object to update based on commands.
        """
        self.robot    = robot
        self.state    = state
        self.keyboard = robot.getKeyboard()
        
        # Enable keyboard with 10x the simulation timestep for responsive input
        timestep_ms = int(robot.getBasicTimeStep())
        self.keyboard.enable(10 * timestep_ms)

    # -------------------------------------------------------------- #
    # Main dispatcher
    # -------------------------------------------------------------- #
    def process_input(self, key: int):
        """
        Process a key press and dispatch to appropriate command handler.
        
        Key mappings:
        - T: Takeoff
        - END: Land
        - G: Toggle gimbal stabilization
        - I/K: Pitch gimbal up/down
        - J/L: Roll gimbal left/right
        - U/O: Yaw gimbal left/right
        - W/S: Move forward/backward
        - A/D: Move left/right
        - LEFT/RIGHT: Yaw left/right
        - UP/DOWN: Increase/decrease altitude
        - HOME: Return to home position
        - M: Toggle ArUco tracking
        - PAGE_DOWN: Return to home at higher altitude (stage A)
        
        Parameters:
            key (int): Key code from Webots keyboard device.
        """
        kb = Keyboard
        if key == ord('T'):             self._takeoff()
        elif key == kb.END:             self._land()
        elif key == ord('G'):           self._toggle_gimbal()
        elif key == ord('I'):           self._gimbal_pitch(+0.005)
        elif key == ord('K'):           self._gimbal_pitch(-0.005)
        elif key == ord('J'):           self._gimbal_roll(+0.005)
        elif key == ord('L'):           self._gimbal_roll(-0.005)
        elif key == ord('U'):           self._gimbal_yaw(+0.005)
        elif key == ord('O'):           self._gimbal_yaw(-0.005)
        elif key == ord('W'):           self._move_x(+0.1)
        elif key == ord('S'):           self._move_x(-0.1)
        elif key == ord('A'):           self._move_y(+0.1)
        elif key == ord('D'):           self._move_y(-0.1)
        elif key == kb.LEFT:            self._move_yaw(+0.05)
        elif key == kb.RIGHT:           self._move_yaw(-0.05)
        elif key == kb.UP:              self._move_z(+0.05)
        elif key == kb.DOWN:            self._move_z(-0.05)
        elif key == kb.HOME:            self._toggle_home()
        elif key == ord('M'):           self._toggle_aruco()
        elif key == kb.PAGEDOWN:        self._rth_stage_a()

    # -------------------------------------------------------------- #
    # Command handlers  (mirrors legacy script)
    # -------------------------------------------------------------- #
    def _takeoff(self):
        """Initiate takeoff sequence: enable motors, set initial altitude and gimbal position."""
        self.state.status_takeoff = True
        self.state.status_gimbal  = True
        self.state.z_target       = 3.0
        self.state.pitch_angle_gimbal = 1.6
        print(">> Take‑off")
        sleep(0.15)  # Debounce delay

    def _land(self):
        """Initiate landing sequence: set altitude target to ground level."""
        self.state.status_landing = True
        self.state.z_target       = 0.1
        print(">> Landing")
        sleep(0.15)  # Debounce delay

    def _toggle_gimbal(self):
        """Toggle gimbal stabilization on/off with reset to default angles when enabled."""
        self.state.status_gimbal = not self.state.status_gimbal
        if self.state.status_gimbal:
            self.state.roll_angle_gimbal  = 0.0
            self.state.pitch_angle_gimbal = 1.6
            self.state.yaw_angle_gimbal   = 0.0
        print(f">> Gimbal stabilise: {self.state.status_gimbal}")
        sleep(0.15)  # Debounce delay

    # --- gimbal fine control -------------------------------------- #
    def _gimbal_pitch(self, delta): self._inc_attr("pitch_angle_gimbal", delta, -0.5, 1.7, "Gimbal pitch")
    def _gimbal_roll (self, delta): self._inc_attr("roll_angle_gimbal",  delta, -0.5, 0.5,  "Gimbal roll")
    def _gimbal_yaw  (self, delta): self._inc_attr("yaw_angle_gimbal",   delta, -1.7, 1.7,  "Gimbal yaw")

    # --- motion set‑points ---------------------------------------- #
    def _move_x(self, d): self._inc_attr("x_target", d, info="Target X")
    def _move_y(self, d): self._inc_attr("y_target", d, info="Target Y")
    def _move_z(self, d): self._inc_attr("z_target", d, info="Target Z")
    def _move_yaw(self, d): self._inc_attr("yaw_target", d, info="Target yaw")

    # --- modes ----------------------------------------------------- #
    def _toggle_home(self):
        """Toggle return-to-home mode: center XY, reset yaw, and increase altitude."""
        self._toggle_aruco()
        self.state.status_home = not self.state.status_home
        if self.state.status_home:
            self.state.x_target = 0.0
            self.state.y_target = 0.0
            self.state.yaw_target = 0.0
            self.state.z_target   = 10.0
        print(f">> Return‑home mode: {self.state.status_home}")
        sleep(0.15)  # Debounce delay

    def _toggle_aruco(self):
        """Toggle ArUco marker tracking mode."""
        self.state.status_aruco = not self.state.status_aruco
        print(f">> ArUco mode: {self.state.status_aruco}")
        sleep(0.15)  # Debounce delay

    def _rth_stage_a(self):
        """Stage A of return-to-home: reset yaw and increase altitude to safe level."""
        self.state.status_home_A = not self.state.status_home_A
        self.state.status_aruco  = not self.state.status_aruco
        if self.state.status_home_A:
            self.state.yaw_target = 0.0
            self.state.z_target   = 20.0
        print(f">> RTH Stage‑A: {self.state.status_home_A}")
        sleep(0.15)  # Debounce delay

    # -------------------------------------------------------------- #
    # Utility
    # -------------------------------------------------------------- #
    def _inc_attr(self, attr, delta, min_v=None, max_v=None, info=None):
        """
        Increment a state attribute with optional range clamping.
        
        Parameters:
            attr (str): Name of the state attribute to modify.
            delta (float): Amount to increment/decrement the attribute by.
            min_v (float, optional): Minimum allowed value.
            max_v (float, optional): Maximum allowed value.
            info (str, optional): Label to print with the updated value.
        """
        val = getattr(self.state, attr) + delta
        if min_v is not None: val = max(min_v, val)
        if max_v is not None: val = min(max_v, val)
        setattr(self.state, attr, val)
        if info: print(f"{info}: {val:+.2f}")