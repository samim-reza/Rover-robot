# Mars Rover Project 🚀

A comprehensive autonomous Mars rover system developed for university competition, featuring advanced navigation, robotic arm control, computer vision, and real-time telemetry monitoring.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Project Structure](#project-structure)
- [Technologies Used](#technologies-used)
- [Installation](#installation)
- [Usage](#usage)
- [Modules](#modules)
- [Hardware Components](#hardware-components)
- [Contributing](#contributing)

## 🌟 Overview

This project represents a full-stack Mars rover system designed for autonomous navigation, object detection, robotic manipulation, and scientific data collection. The system integrates multiple subsystems including GPS navigation, magnetometer-based heading control, camera streaming, robotic arm control, and comprehensive dashboard monitoring.

## ✨ Features

### Core Capabilities
- **Autonomous Navigation**: GPS waypoint-based navigation with magnetometer heading control
- **Real-time Video Streaming**: Multiple IP camera feeds with object detection
- **Robotic Arm Control**: Precision arm manipulation for sample collection
- **Advanced Dashboard**: Multiple React-based dashboards for mission monitoring
- **ROS Integration**: Full ROS (Robot Operating System) support with ROSBridge
- **Computer Vision**: YOLO-based object detection and OCR text recognition
- **Sensor Integration**: IMU, GPS, magnetometer, temperature, and humidity sensors
- **Panoramic Imaging**: Automated panorama capture and stitching
- **3D Visualization**: STL model viewer for rover design visualization

### Control Systems
- **Manual Control**: Joystick and web-based remote control interfaces
- **Autonomous Mode**: Waypoint navigation with obstacle avoidance
- **Actuator Control**: Stepper motor and linear actuator control
- **Motor Control**: ODrive-based BLDC motor control with encoder feedback

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Web Dashboards                        │
│  (React + TypeScript + Vite + Bootstrap + Leaflet)      │
└────────────────┬────────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────────┐
│              Backend Services (Flask/Express)            │
│  - Video Streaming   - WebSocket Control                │
│  - Sensor Data       - Command Processing               │
└────────────────┬────────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────────┐
│                 ROS Middleware                           │
│  - ROSBridge     - Navigation Stack                     │
│  - URDF Models   - Sensor Integration                   │
└────────────────┬────────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────────┐
│          Hardware Control Layer (Python/RPi)            │
│  - Motor Drivers  - GPS         - Magnetometer          │
│  - Cameras        - Sensors     - Actuators             │
└─────────────────────────────────────────────────────────┘
```

## 📁 Project Structure

```
Rover-robot/
│
├── autonomous/              # Autonomous navigation dashboard (React + TypeScript)
│   ├── src/
│   │   ├── components/     # React components
│   │   └── App.tsx         # Main application
│   ├── app.py             # Flask backend for video streaming
│   └── package.json
│
├── RoversDashboard/        # Main control dashboard
│   └── rover_dashboard/
│       ├── src/
│       │   ├── components/
│       │   │   ├── AutonomousDashboard/
│       │   │   ├── ArmsDashboard/
│       │   │   └── ScienceDashboard/
│       │   └── App.tsx
│       └── package.json
│
├── carcontrol/            # Vehicle control interface
│   ├── index.html         # Main control page
│   ├── sensor.html        # Sensor monitoring page
│   ├── armcontrol/        # Robotic arm control
│   └── server.js          # Express server
│
├── ipcam/                 # IP camera streaming module
│   ├── app.py             # Flask video server
│   ├── camera-stream/     # React camera viewer
│   ├── stl_files/         # 3D models
│   └── server.py          # STL file server
│
├── pi/                    # Raspberry Pi control scripts
│   ├── rover.py           # Main rover control (GPS + Magnetometer)
│   ├── kalman_rover.py    # Kalman filter implementation
│   ├── magnetometer.py    # Magnetometer interface
│   └── arduino_reader.py  # Arduino communication
│
├── joystick/              # Joystick control
│   ├── joystick.py        # Joystick interface (Pygame)
│   ├── yolo-app.py        # Object detection with YOLO
│   └── pi_arm.py          # Arm control via joystick
│
├── odrive/                # Motor control (ODrive)
│   ├── odrive_control.py  # ODrive initialization
│   ├── control_motor.py   # Motor control functions
│   └── torque.py          # Torque mode control
│
├── catkin_ws/             # ROS workspace
│   └── src/
│       ├── my_robot_description/  # URDF models
│       ├── pi/            # ROS nodes for Pi
│       └── robot/         # Robot control packages
│
├── Rosbridge/             # ROS web integration
│   └── rover/             # React dashboard with ROS
│
├── OCR/                   # Optical Character Recognition
│   ├── textDetect.py      # Real-time text detection
│   └── ytCode.py          # Video processing
│
├── stepper/               # Stepper motor control
│   └── stepper-actuator/  # Linear actuator interface
│
├── position/              # GPS position visualization
│   ├── index.html         # Position display
│   └── up.html            # Position upload
│
├── Map-start/             # Mapping interface
│   ├── first-page.html    # Landing page
│   └── graph.html         # Data visualization
│
├── View-page/             # Data visualization
│   ├── view.html          # Sensor data display
│   └── dummy-data.js      # Mock data server
│
├── panorama_capture.py    # Panoramic image capture
└── index.html            # Main navigation interface
```

## 🛠️ Technologies Used

### Frontend
- **React 18/19** - UI framework
- **TypeScript** - Type-safe JavaScript
- **Vite** - Build tool and dev server
- **Bootstrap 5** - UI components and styling
- **Leaflet** - Interactive maps
- **Chart.js / Recharts / Plotly.js** - Data visualization
- **React Router** - Navigation
- **Styled Components** - CSS-in-JS

### Backend
- **Flask** - Python web framework
- **Express.js** - Node.js web framework
- **Flask-CORS** - Cross-origin resource sharing
- **WebSocket** - Real-time communication

### Computer Vision & AI
- **OpenCV** - Image processing
- **YOLO (Ultralytics)** - Object detection
- **Tesseract OCR** - Text recognition
- **NumPy** - Numerical computing

### Robotics
- **ROS (Robot Operating System)** - Robotics middleware
- **ROSBridge** - ROS web integration
- **URDF** - Robot description format
- **Catkin** - ROS build system

### Hardware Control
- **gpiozero** - Raspberry Pi GPIO control
- **smbus** - I2C communication
- **pyserial** - Serial communication
- **pynmea2** - GPS data parsing
- **ODrive** - Motor controller library
- **Pygame** - Joystick interface

### DevOps & Tools
- **Git** - Version control
- **ESLint** - Code linting
- **npm/yarn** - Package management
- **Python pip** - Python package management

## 📦 Installation

### Prerequisites
```bash
# System requirements
- Node.js (v18+)
- Python 3.8+
- ROS Noetic (Ubuntu 20.04)
- Raspberry Pi 4 (for hardware control)
```

### Clone Repository
```bash
git clone https://github.com/samim-reza/Rover-robot.git
cd Rover-robot
```

### Frontend Setup

#### Autonomous Dashboard
```bash
cd autonomous
npm install
npm run dev
# Server runs on http://localhost:5173
```

#### Rovers Dashboard
```bash
cd RoversDashboard/rover_dashboard
npm install
npm run dev
```

#### Camera Stream
```bash
cd ipcam/camera-stream
npm install
npm start
```

#### ROSBridge Dashboard
```bash
cd Rosbridge/rover
npm install
npm run dev
```

### Backend Setup

#### Flask Servers
```bash
# Install Python dependencies
pip install flask flask-cors opencv-python opencv-contrib-python
pip install ultralytics pytesseract pynmea2 gpiozero smbus2 odrive

# Run autonomous video server
cd autonomous
python app.py
# Server runs on http://localhost:5000

# Run IP camera server
cd ipcam
python app.py

# Run YOLO detection server
cd joystick
python yolo-app.py
```

#### Express Servers
```bash
# Install Node dependencies
npm install express cors

# Run car control server
cd carcontrol
node server.js
# Server runs on http://localhost:3000

# Run remote control server
cd remote
node server.js

# Run data server
cd View-page
node dummy-data.js
```

### ROS Setup
```bash
cd catkin_ws
catkin_make
source devel/setup.bash

# Launch ROS nodes
roslaunch my_robot_description display.launch
```

### Hardware Setup (Raspberry Pi)
```bash
# Enable I2C and Serial
sudo raspi-config
# Navigate to Interfacing Options -> Enable I2C and Serial

# Install Python dependencies
pip install gpiozero smbus2 pyserial pynmea2

# Run rover control
cd pi
python rover.py
```

## 🚀 Usage

### Starting the System

1. **Start Backend Services**
```bash
# Terminal 1: Autonomous navigation server
cd autonomous && python app.py

# Terminal 2: Car control server
cd carcontrol && node server.js

# Terminal 3: Data visualization server
cd View-page && node dummy-data.js
```

2. **Start Frontend Dashboards**
```bash
# Terminal 4: Main dashboard
cd RoversDashboard/rover_dashboard && npm run dev

# Terminal 5: Autonomous dashboard
cd autonomous && npm run dev
```

3. **Launch ROS (if using)**
```bash
# Terminal 6: ROS core and nodes
roscore &
roslaunch my_robot_description display.launch
```

4. **Start Rover Control (on Raspberry Pi)**
```bash
cd pi
python rover.py
```

### Access Points

| Service | URL | Description |
|---------|-----|-------------|
| Main Dashboard | http://localhost:5173 | Primary control interface |
| Autonomous Dashboard | http://localhost:5174 | Navigation monitoring |
| Car Control | http://localhost:3000 | Manual control interface |
| Camera Stream | http://localhost:5000/video_feed | Live video feed |
| Data Visualization | http://localhost:3000 | Sensor data display |
| ROS Visualization | http://localhost:5175 | ROSBridge interface |

## 📱 Modules

### 1. Autonomous Navigation (`autonomous/`)
- **Purpose**: GPS-based waypoint navigation with real-time monitoring
- **Features**: 
  - Live GPS tracking on Leaflet map
  - Heading and distance calculations
  - Mission timeline and notifications
  - Camera feed integration
  - Autonomous mode control

### 2. Rovers Dashboard (`RoversDashboard/`)
- **Purpose**: Comprehensive mission control center
- **Components**:
  - **Autonomous Dashboard**: Navigation and path planning
  - **Arms Dashboard**: Robotic arm control and monitoring
  - **Science Dashboard**: Scientific instrument data
- **Features**: Multi-tab interface, real-time notifications, system mode switching

### 3. Car Control (`carcontrol/`)
- **Purpose**: Manual vehicle and arm control
- **Features**:
  - Directional control (Forward, Backward, Left, Right)
  - Arm joint control
  - Sensor monitoring page
  - WebSocket-based real-time control

### 4. IP Camera Module (`ipcam/`)
- **Purpose**: Video streaming and 3D model visualization
- **Features**:
  - MJPEG stream capture
  - Flask video server
  - React-based viewer
  - STL model viewer for rover design
  - CORS-enabled API

### 5. Raspberry Pi Control (`pi/`)
- **Purpose**: Low-level hardware control
- **Key Scripts**:
  - `rover.py`: Main navigation loop with GPS and magnetometer
  - `kalman_rover.py`: Kalman filter for sensor fusion
  - `magnetometer.py`: HMC5883L magnetometer interface
  - `arduino_reader.py`: Arduino sensor data collection

### 6. Joystick Control (`joystick/`)
- **Purpose**: Game controller integration
- **Features**:
  - Pygame-based joystick interface
  - Dead zone configuration
  - YOLO object detection integration
  - Arm control via joystick

### 7. ODrive Motor Control (`odrive/`)
- **Purpose**: High-performance motor control
- **Features**:
  - Motor calibration scripts
  - Encoder configuration
  - Velocity and position control
  - Torque mode operation

### 8. ROS Integration (`catkin_ws/`)
- **Purpose**: Robot Operating System integration
- **Packages**:
  - `my_robot_description`: URDF models and visualization
  - `pi`: ROS nodes for Raspberry Pi
  - `robot`: Core robot control packages

### 9. Computer Vision (`OCR/`, `joystick/yolo-app.py`)
- **Purpose**: Vision-based capabilities
- **Features**:
  - Real-time text detection (Tesseract OCR)
  - Object detection (YOLO)
  - Video processing and analysis

### 10. Panoramic Imaging (`panorama_capture.py`)
- **Purpose**: Wide-angle image capture
- **Features**:
  - Multi-image capture with stabilization
  - Image stitching
  - Automatic rotation detection

### 11. Stepper Motor Control (`stepper/`, `stepper-actuator/`)
- **Purpose**: Precise motor control
- **Features**:
  - WebSocket-based control
  - Linear actuator positioning
  - Real-time command sending

### 12. Position Tracking (`position/`)
- **Purpose**: GPS coordinate management
- **Features**:
  - Coordinate input interface
  - Real-time position updates
  - WebSocket communication

### 13. Mapping Interface (`Map-start/`)
- **Purpose**: Geographic data visualization
- **Features**:
  - Interactive map display
  - Data graphing capabilities
  - Landing page for navigation

### 14. Data Visualization (`View-page/`)
- **Purpose**: Sensor data display
- **Features**:
  - Distance sensors
  - Temperature and humidity
  - IMU data (X, Y, Z axes)
  - PWM motor control values
  - Mock data server for testing

### 15. ROSBridge Interface (`Rosbridge/`)
- **Purpose**: Web-based ROS interaction
- **Features**:
  - Real-time ROS topic visualization
  - Command publishing
  - React-based dashboard

## 🔧 Hardware Components

### Electronics
- **Raspberry Pi 4** - Main computing unit
- **ODrive v3.6** - Motor controller
- **HMC5883L** - 3-axis magnetometer
- **GPS Module** - NEO-6M or similar
- **Arduino** - Auxiliary sensor controller
- **ESP32** - WebSocket control for actuators
- **IMU** - Inertial Measurement Unit
- **Temperature/Humidity Sensor** - DHT22 or similar

### Motors & Actuators
- **BLDC Motors** - Wheel drive motors
- **Stepper Motors** - Precision positioning
- **Linear Actuators** - Sample collection mechanism
- **Servo Motors** - Arm joint control

### Sensors
- **IP Cameras** - Multiple camera feeds
- **Ultrasonic Sensors** - Distance measurement
- **Encoders** - Motor position feedback
- **Compass** - Magnetic heading

### Power
- **LiPo Batteries** - Main power source
- **Voltage Regulators** - 5V/3.3V rails
- **Power Distribution Board**

## 🎯 Key Features Explained

### GPS Navigation System
The rover uses a sophisticated GPS navigation system with:
- Multiple waypoint support (target coordinates)
- Real-time heading calculation using magnetometer
- Distance-based decision making
- PID-like control for accurate navigation
- Magnetic declination compensation

### Motor Control Strategy
```python
# Differential drive control
- Left motors (EN1, IN1, IN2)
- Right motors (EN2, IN3, IN4)
- Speed adjustment: 0-1 (PWM)
- Direction control: Forward/Backward/Turn
```

### Camera Streaming Architecture
- MJPEG stream capture from IP cameras
- Flask server-side encoding
- Real-time browser display
- Multiple camera support
- Authentication handling

### Dashboard Features
- **Real-time Updates**: WebSocket connections
- **Responsive Design**: Bootstrap grid system
- **Interactive Maps**: Leaflet integration
- **Data Visualization**: Chart.js graphs
- **Notifications**: Toast-style alerts
- **Mode Switching**: Manual/Autonomous toggle

## 🐛 Troubleshooting

### Camera Issues
```bash
# Check camera connection
v4l2-ctl --list-devices

# Test camera stream
ffplay -i "http://admin:admin123@192.168.1.112/video.cgi"
```

### GPS Issues
```bash
# Check serial port
ls -l /dev/ttyAMA0

# Test GPS data
cat /dev/ttyAMA0
```

### I2C Issues
```bash
# Scan I2C devices
i2cdetect -y 1

# Expected: Magnetometer at 0x1E
```

### Node.js Issues
```bash
# Clear npm cache
npm cache clean --force

# Reinstall dependencies
rm -rf node_modules package-lock.json
npm install
```

### Python Issues
```bash
# Upgrade pip
pip install --upgrade pip

# Install missing dependencies
pip install -r requirements.txt
```

## 📝 Configuration

### Network Configuration
Update IP addresses in the following files:
- `autonomous/app.py` - Camera stream URL
- `carcontrol/server.js` - Server port
- `actuator/index.html` - ESP32 IP
- `stepper/index2.html` - WebSocket address

### GPS Waypoints
Edit waypoints in `pi/rover.py`:
```python
target_lat = [23.8296169, 23.82952, ...]
target_lng = [90.5672889, 90.567155, ...]
```

### Motor Parameters
Adjust in `pi/rover.py`:
```python
SPEED = 0.7       # Maximum speed (0-1)
L_R_SPD = 0.3     # Turn speed
DECLINATION = 0.22 # Magnetic declination for your location
```

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is developed for educational purposes as part of a university Mars rover competition.

## 👥 Team

Developed by university students for Mars rover competition.

## 🙏 Acknowledgments

- ROS community for robotics frameworks
- OpenCV for computer vision capabilities
- React community for modern web development tools
- Leaflet for mapping capabilities
- All open-source contributors

## 📞 Contact

For questions or collaboration:
- GitHub: [@samim-reza](https://github.com/samim-reza)
- Repository: [Rover-robot](https://github.com/samim-reza/Rover-robot)

## 🗺️ Roadmap

### Current Features
- ✅ GPS navigation
- ✅ Camera streaming
- ✅ Robotic arm control
- ✅ Web dashboards
- ✅ ROS integration
- ✅ Object detection

### Planned Features
- 🔄 Advanced path planning algorithms
- 🔄 SLAM (Simultaneous Localization and Mapping)
- 🔄 Enhanced obstacle avoidance
- 🔄 Machine learning for terrain classification
- 🔄 Improved sensor fusion
- 🔄 Mobile app for remote control

---

**Note**: This is an active development project. Some features may be in experimental stages. Always test in a safe environment before deploying on hardware.

**Built with ❤️ for Mars exploration simulation**
