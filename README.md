# AiNex

English | [中文](https://github.com/Hiwonder/ainex/blob/main/README_cn.md)

<p align="center">
  <img src="./sources/images/ainex.png" alt="AiNex Logo" width="600"/>
</p>

## Product Overview

AiNex is an advanced quadruped robot platform developed by Hiwonder, powered by ROS (Robot Operating System). Featuring sophisticated kinematics control, sensor integration, and AI capabilities, AiNex provides a comprehensive platform for robotics research, education, and development.

## Official Resources

### Official Hiwonder

- **Official Website**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **Product Page**: [https://www.hiwonder.com/products/ainex](https://www.hiwonder.com/products/ainex)
- **Official Documentation**: [https://docs.hiwonder.com/projects/AiNex/en/latest/](https://docs.hiwonder.com/projects/AiNex/en/latest/)
- **Technical Support**: support@hiwonder.com

## Key Features

### ROS Integration

- **ROS-based Architecture** - Built on Robot Operating System for modularity and flexibility
- **Multiple Launch Files** - Easy startup and configuration management
- **Service Integration** - Systemd services for automatic startup
- **Rosbridge Support** - WebSocket interface for remote control

### Advanced Control

- **Kinematics Control** - Precise inverse kinematics for smooth motion
- **Servo Controller** - High-precision servo motor control system
- **Walking Parameters** - Configurable gait parameters for different terrains
- **Pose Management** - Initial pose configuration and management

### Sensor Calibration

- **IMU Calibration** - Inertial Measurement Unit calibration tools
- **Magnetometer Calibration** - Magnetic sensor calibration support
- **Camera Calibration** - Vision system calibration utilities

### Application Support

- **Mobile APP Integration** - Remote control via smartphone application
- **OLED Display** - Real-time status display on OLED screen
- **Custom Scripts** - Extensible Python-based scripting system

## Repository Structure

```
ainex/
└── src/
    ├── ainex_app/              # Application layer
    │   ├── launch/             # Launch files
    │   └── scripts/            # Application scripts
    ├── ainex_bringup/          # System startup
    │   ├── launch/             # Bringup launch files
    │   ├── scripts/            # Startup scripts
    │   └── service/            # Systemd services
    ├── ainex_calibration/      # Calibration tools
    │   ├── config/             # Calibration configs
    │   └── launch/             # Calibration launch files
    └── ainex_driver/           # Hardware drivers
        └── ainex_kinematics/   # Kinematics control
            ├── config/         # Controller configs
            └── launch/         # Driver launch files
```

## Getting Started

### Prerequisites

- ROS Noetic (or compatible version)
- Python 3.x
- Required ROS packages

### Installation

1. Clone the repository:
```bash
git clone https://github.com/Hiwonder/ainex.git
cd ainex
```

2. Build the workspace:
```bash
catkin_make
source devel/setup.bash
```

3. Launch the system:
```bash
roslaunch ainex_bringup bringup.launch
```

## Usage

### Basic Control

Launch the base controller:
```bash
roslaunch ainex_bringup base.launch
```

### Calibration

Run IMU calibration:
```bash
roslaunch ainex_calibration imu_calibration.launch
```

Run camera calibration:
```bash
roslaunch ainex_calibration camera_calibration.launch
```

### Application Mode

Start the application node:
```bash
roslaunch ainex_app start.launch
```

## Community & Support

- **GitHub Issues**: Report bugs and request features
- **Email Support**: support@hiwonder.com
- **Documentation**: Comprehensive guides and tutorials

## License

This project is open-source and available for educational and research purposes.

---

**Hiwonder** - Empowering Innovation in Robotics Education
