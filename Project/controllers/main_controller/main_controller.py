from controller import Robot
import numpy as np
import cv2

from mavic_toolkit import (
    SensorSuite,
    FlightController,
    GimbalController,
    ArucoDetector,
    PIDController,
    State,
    ControlParams,
    KeyboardHandler,
)

# --------------------------------------------------------------------------- #
# Utility Functions
# --------------------------------------------------------------------------- #
def clamp(val: float, vmin: float, vmax: float) -> float:
    """
    Clamp a value between a minimum and maximum boundary.

    Parameters:
        val (float): The input value to be clamped.
        vmin (float): The minimum allowable value.
        vmax (float): The maximum allowable value.

    Returns:
        float: The clamped value within [vmin, vmax].
    """
    return max(min(val, vmax), vmin)


# --------------------------------------------------------------------------- #
# ArucoDroneController
# --------------------------------------------------------------------------- #
class ArucoDroneController(Robot):
    """
    High-level drone controller integrating sensors, PID loops, and ArUco-based
    auto-landing logic. Subclasses the Webots Robot API.

    Attributes:
        timestep (int): Control loop step time in milliseconds.
        state (State): Holds flight targets and status flags.
        config (ControlParams): Holds static control parameters.
        sensors (SensorSuite): Manages IMU, GPS, camera, etc.
        flt_ctl (FlightController): Sends velocity commands to motors.
        gimbal_ctl (GimbalController): Adjusts camera gimbal orientation.
        aruco (ArucoDetector): Detects ArUco markers in camera frames.
        pid_ctl (PIDController): PID loops for altitude and heading.
        keyboard_hdl (KeyboardHandler): Processes user keyboard inputs.
    """

    # Baseline vertical thrust in rad/s (matches DJI Mavic 2)
    K_VERTICAL_THRUST = ControlParams().vertical_thrust

    def __init__(self):
        """
        Initialize subsystems, enable sensors, and configure the debug window.
        """
        super().__init__()
        # Convert Webots time step to int for consistent use
        self.timestep = int(self.getBasicTimeStep())

        # Instantiate state and config containers
        self.state = State()
        self.config = ControlParams()

        # Initialize hardware abstraction layers
        self.sensors = SensorSuite(self)
        self.flt_ctl = FlightController(self)
        self.gimbal_ctl = GimbalController(self)
        self.aruco = ArucoDetector()
        self.pid_ctl = PIDController()
        self.keyboard_hdl = KeyboardHandler(self, self.state)

        # Enable all sensors at the requested time step
        self.sensors.enable_all()

        # Configure a resizable OpenCV window for camera preview
        cv2.namedWindow("Mavic-2 Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Mavic-2 Camera", 800, 480)

    def run(self):
        """
        Main control loop:
          1. Read sensors
          2. Handle user input
          3. Process ArUco detection and update targets
          4. Execute PID control and motor mixing
          5. Display camera feed for debugging
        """
        while self.step(self.timestep) != -1:
            # 1. Sense: position (xyz), orientation (rpy), angular rates, camera frame
            pos = self.sensors.get_position()
            rpy = self.sensors.get_orientation()
            gyro = self.sensors.gyro.getValues()
            frame = self.sensors.get_camera_frame()

            # 2. User commands: takeoff, land, gimbal, movement, modes
            key = self.keyboard_hdl.keyboard.getKey()
            if key != -1:
                self.keyboard_hdl.process_input(key)

            # 3. ArUco mission logic: auto-align and landing trigger
            self._update_state_with_aruco(frame, pos)

            # 4. Closed-loop flight control
            self._control_loop(pos, rpy, gyro)

            # 5. Visual debug: show BGR camera frame (no alpha channel)
            cv2.imshow("Mavic-2 Camera", frame)
            cv2.waitKey(1)

    def _update_state_with_aruco(self, frame, pos):
        """
        Detect ArUco marker, compute pixel misalignment,
        convert to XY error targets, and trigger auto-landing.

        Parameters:
            frame (ndarray): BGR camera frame (H x W x 3).
            pos (array): Current [x, y, z] GPS position (m).
        """
        h, w, _ = frame.shape
        cx, cy = w / 2, h / 2

        # Detect and draw markers
        corners, ids, _, vis = self.aruco.detect(frame)
        frame[:] = vis  # In-place overlay

        if self.state.status_aruco and ids is not None:
            # Compute center of first detected marker
            marker = corners[0][0]
            center_x = marker[1][0] + (marker[0][0] - marker[1][0]) / 2
            center_y = marker[2][1] + (marker[0][1] - marker[2][1]) / 2

            # Convert pixel offset to ±4m error in X/Y
            self.state.y_error_img = 4 * ((center_x - cx) / w)
            self.state.x_error_img = -4 * ((center_y - cy) / h)

            # If aligned within 0.1m and altitude within 0.5m, trigger landing
            alti_err = pos[2] - self.state.z_target
            if (abs(self.state.x_error_img) < 0.1 and
                abs(self.state.y_error_img) < 0.1 and
                abs(alti_err) < 0.5 and
                not self.state.status_landing):
                self.state.status_landing = True
                self.state.z_target = 0.0
                print(">> Auto-landing triggered")
        else:
            # Reset vision error if ArUco is disabled or lost
            self.state.x_error_img = None
            self.state.y_error_img = None

    def _control_loop(self, pos, rpy_deg, gyro_rads):
        """
        Compute PID outputs and mix into motor commands.

        Parameters:
            pos (array): [x, y, z] position (m).
            rpy_deg (tuple): Roll/Pitch/Yaw angles (degrees).
            gyro_rads (tuple): Angular rates (rad/s).
        """
        # Convert degrees to radians for internal control
        roll, pitch, yaw = np.radians(rpy_deg)
        roll_acc, pitch_acc, yaw_acc = gyro_rads
        x, y, z = pos

        # Select error source: vision or manual setpoints
        if self.state.status_aruco and self.state.x_error_img is not None:
            roll_err = clamp(-self.state.y_error_img + 0.06, -1.5, 1.5)
            pitch_err = clamp(self.state.x_error_img - 0.13, -1.5, 1.5)
        else:
            roll_err  = clamp(-(y - 0.06) + self.state.y_target, -1.5, 1.5)
            pitch_err = clamp(-(x + 0.13) + self.state.x_target, -1.5, 1.5)

        # Set PID setpoints for yaw and altitude
        self.pid_ctl.setpoints(yaw=self.state.yaw_target,
                               z=self.state.z_target)

        # Compute outer-loop PID outputs
        yaw_out = self.pid_ctl['yaw'](yaw)
        z_out   = self.pid_ctl['z'](z)

        # Mix in fast attitude PIDs and errors
        roll_in = (self.pid_ctl.params.roll[0] * clamp(roll, -1, 1) +
                   self.pid_ctl.params.roll[2] * roll_acc +
                   roll_err)
        pitch_in = (self.pid_ctl.params.pitch[0] * clamp(pitch, -1, 1) +
                    self.pid_ctl.params.pitch[2] * pitch_acc -
                    pitch_err)

        # Motor mixing: identical to original Mavic formula
        fl = self.K_VERTICAL_THRUST + z_out - yaw_out + pitch_in - roll_in
        fr = self.K_VERTICAL_THRUST + z_out + yaw_out + pitch_in + roll_in
        rl = self.K_VERTICAL_THRUST + z_out + yaw_out - pitch_in - roll_in
        rr = self.K_VERTICAL_THRUST + z_out - yaw_out - pitch_in + roll_in

        # If grounded or landing complete, disable motors
        if (not self.state.status_takeoff or
            (self.state.status_landing and z <= 0.1)):
            fl = fr = rl = rr = 0.0
            self.state.status_takeoff = False
            self.state.status_landing = False
            self.state.status_gimbal  = False
            self.state.status_home    = False

        # Send commands to props
        self.flt_ctl.set_motor_speeds(fl, fr, rl, rr)

        # Stabilize gimbal if enabled
        if self.state.status_gimbal:
            roll_gb  = clamp(-0.001 * roll_acc + self.state.roll_angle_gimbal,
                             -0.5, 0.5)
            pitch_gb = clamp(-0.001 * pitch_acc + self.state.pitch_angle_gimbal,
                             -0.5, 1.7)
            yaw_gb   = clamp(-0.001 * yaw_acc + self.state.yaw_angle_gimbal,
                             -1.7, 1.7)
            self.gimbal_ctl.set_orientation(roll_gb, pitch_gb, yaw_gb)


# --------------------------------------------------------------------------- #
# Entrypoint
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    """Run the controller when invoked as a script."""
    ArucoDroneController().run()
