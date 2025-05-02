# 🛩️ Mavic Simulation with ArUco-based Auto-Landing in Webots

A full simulation environment for a DJI Mavic 2–style quadcopter using [Webots](https://cyberbotics.com/) and Python, enabling autonomous landing based on **ArUco marker detection**. Includes camera tracking, PID-based control loops, gimbal stabilization, and keyboard interaction.

> Developed using Webots + OpenCV + Python + ArUco detection for aerial robotics experimentation and education.

---

## 🎯 Features

- ✅ **Quadcopter simulation** using Webots physics engine
- 🎯 **ArUco marker detection** for visual target landing
- 🔁 **PID control loops** for roll, pitch, yaw, and altitude
- 🎥 **Camera + gimbal stabilization** with real-time tracking
- ⌨️ **Keyboard control** for takeoff, landing, gimbal, and home-return
- 🖼️ **OpenCV GUI** for live vision debugging
- 🗺️ Custom `mavic_world.wbt` with ArUco landing zone

---

## 📁 Project Structure

```

Mavic_Webots_Aruco/
├── LICENSE
├── README.md
├── controllers/
│   └── main_controller/
│       ├── main_controller.py       # Main simulation controller
│       └── mavic_toolkit/           # Modular control framework
│           ├── actuators.py         # Motor and gimbal control logic
│           ├── command.py           # Keyboard interaction handler
│           ├── config.py            # PID gains and flight constants
│           ├── markers.py           # ArUco marker detection
│           ├── pid_controller.py    # Custom PID implementation
│           ├── sensors.py           # IMU, GPS, camera, gyro access
│           └── **init**.py
└── worlds/
├── mavic_world.wbt              # Webots world with landing pad
└── assets/
└── aruco-land.png           # Landing target image

```

---

## 🧠 How It Works

### 🔹 Sensor Suite
- Reads **GPS**, **IMU**, **gyro**, and **camera** data using Webots API
- Fetches camera frames in OpenCV-compatible format (BGR)

### 🔹 ArUco Marker Detection
- Uses OpenCV `cv2.aruco` to detect landing pad marker
- Computes pixel offset from image center → mapped to XY position error
- Triggers **auto-landing** if aligned and altitude is below threshold

### 🔹 PID Flight Control
- Outer-loop PID controls **altitude** and **heading**
- Attitude stabilization uses roll/pitch errors and angular velocity
- Rotor speeds are mixed using a DJI-like formula

### 🔹 Keyboard Shortcuts (via `command.py`)
| Key      | Function             |
|----------|----------------------|
| `T`      | Takeoff              |
| `L`      | Land                 |
| `G`      | Toggle Gimbal Lock   |
| `Home`   | Return to Home       |
| Arrows   | Manual Movement      |

---

## ▶️ Usage Instructions

### ✅ Requirements
- [Webots](https://cyberbotics.com/) R2023a or newer
- Python 3.8+
- Packages: `opencv-python`, `numpy`

```bash
pip install opencv-python numpy
```

### 🚀 Run the Simulation

1. Launch Webots and open `worlds/mavic_world.wbt`
2. Make sure your Python controller is set to: `main_controller`
3. Press **Play** ▶️ to start the simulation
4. Use keyboard commands to control flight and observe behavior

> 🖼️ A live OpenCV window will show ArUco marker detection feedback.

---

## 🖼️ ArUco Marker for Landing

You can find the landing pad ArUco marker in:

```
worlds/assets/aruco-land.png
```

You can generate your own markers using:

```python
import cv2
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.drawMarker(aruco_dict, id=1, sidePixels=200)
```

---

## 📸 Sample Output

| ArUco View           | Altitude View      |
| -------------------- | ------------------ |
| *Live OpenCV window* | *Z = 0.0 → landed* |

---

## 🔧 Extending the Simulation

* 📦 Add **trajectory tracking** logic using waypoints
* 🎮 Integrate joystick or RC-style controller using `pygame`
* 🔁 Train ArUco alignment using reinforcement learning or CNN
* 📷 Add additional sensors: LiDAR, depth, stereo, etc.

---

## 🧑‍💻 Author

**Ardy Seto Priambodo**
📬 [2black0@gmail.com](mailto:2black0@gmail.com)

---

## 📄 License

Distributed under the [MIT License](LICENSE)

---

## ✨ Acknowledgments

* [Cyberbotics Webots](https://cyberbotics.com/)
* [OpenCV ArUco Module](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
* DJI Mavic 2 parameters adapted for realism

---

> 🛬 *Land like a pro—with vision and control.*