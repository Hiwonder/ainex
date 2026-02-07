# AiNex

English | [中文](https://github.com/Hiwonder/ainex/blob/main/README_cn.md)

<p align="center">
  <img src="./sources/ainex 1.png" alt="AiNex Logo" width="600"/>
</p>

## Product Overview

### About AiNex

At the heart of AiNex is a simple yet profound ambition: to transform the sophisticated humanoid robots of research labs into an accessible "companion" for every developer, student, and researcher. It is more than a hardware kit; it is a fully open-source intelligent entity built upon the Robot Operating System (ROS). We believe true innovation happens when algorithms step out of simulation and learn to "stand" and "walk" in the real, physical world. AiNex was created for this purpose—a 24-DOF bipedal robot designed to be your most exciting experimental platform for exploring mobile robotics, computer vision, and artificial intelligence.

<p align="center">
  <img src="./sources/ainex 2.png" alt="AiNex Logo" width="600"/>
</p>

### High-Performance Hardware Designed for Human-Like Intelligence

The core of AiNex's design is to grant it human-like mobility. Powered by a Raspberry Pi 4B/5 as its computational brain and driven by 24 intelligent serial bus servos, it achieves precise and flexible pose control. Bipedal walking, climbing over obstacles, and complex gestures are all within its capabilities. The integrated 2-DOF HD camera and openable dexterous robotic hands form the foundation for its perception and interaction with the world. All of this is housed within a sturdy metal frame, ensuring stability during dynamic tasks.

<p align="center">
  <img src="./sources/ainex 3.png" alt="AiNex Logo" width="600"/>
</p>

### Core Capabilities Powered by Open-Source Algorithms

AiNex's intelligence stems from its open software stack. Its soul is the advanced inverse kinematics algorithm, enabling omnidirectional movement, flexible gait planning, and real-time pose adjustment for fluid and natural motions like walking, turning, and stair climbing. Combined with OpenCV-based machine vision, AiNex accomplishes advanced AI applications such as precise color/object recognition, visual tracking, line following, and intelligent sorting. From camera capture to cognitive decision-making and coordinated whole-body movement, it forms a complete intelligent loop.

<p align="center">
  <img src="./sources/ainex 4.png" alt="AiNex Logo" width="600"/>
</p>

### Your Project, Your Infinite Possibilities

AiNex is a true development platform. We provide complete ROS source code, extensive tutorials, and a vibrant community to support your journey from basic motion control to cutting-edge AI research. Whether you're exploring reinforcement learning for bipedal locomotion, developing human-robot interaction systems, or prototyping service robot applications, AiNex is ready to bring your vision to life.

## Demo Videos
### Main Tutorials
- **AiNex - ROS AI Educational Walking Humanoid Robot Raspberry Pi Competition Robot**: [Watch](https://www.youtube.com/watch?v=Oizs08q8_9g)

### Your Ideas, In Action
- **When did robots get so quirky? 🎄🤖**: [Watch](https://www.youtube.com/shorts/CXewMZ_aOFM)
- **Watch AiNex effortlessly soaring to snatch the object from a distance**: [Watch](https://www.youtube.com/shorts/EnTP0MkB2ic)
- **AiNex 💪is lifting a dumbbell using its flexible mechanical hands 👉**: [Watch](https://www.youtube.com/shorts/OUJv9BwR1rk)
- **AiNex has a mind-blowing 16DOF flexible body 🤸‍♀️, and now, it's got some pretty nifty hands too✨**: [Watch](https://www.youtube.com/shorts/0nQ7VIA_-94)
  
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
