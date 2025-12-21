# TDK 2025 Computer

A ROS/ROS2 workspace for the TDK 2025 competition, featuring computer vision, mission control, and Intel RealSense camera integration.

## 📋 Overview

This repository contains the software stack for the TDK 2025 computer system, including: 
- ROS2 packages for mission control and computer vision
- Intel RealSense camera integration
- Serial communication workspace (ROS1)
- Docker support for containerized deployment

## 🏗️ Repository Structure

```
tdk2025_computer/
├── packages/
│   ├── ros2-pkg/              # ROS2 packages
│   │   ├── mission_ctrl/      # Mission control logic
│   │   └── opencv_pkg/        # Computer vision with OpenCV
│   └── realsense-pkg/         # Intel RealSense camera drivers and examples
│       └── realsense2_camera/ # RealSense ROS2 wrapper
├── comm-ws/                   # ROS1 Catkin workspace for serial communication
│   ├── src/                   # Source packages
│   └── start_flash.sh         # Flash initialization script
└── docker/                    # Docker configuration files
```

## 🚀 Getting Started

### Prerequisites

- **ROS2** (Humble/Foxy recommended)
- **ROS1** (Noetic for comm-ws)
- **Intel RealSense SDK 2.0**
- **Python 3.x**
- **OpenCV**
- **Docker**

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/chuangtsh/tdk2025_computer.git
cd tdk2025_computer
```

2. **setup with docker**
```bash
cd docker
docker compose up
```
3. **attach to the environment**
```bash
docker exec -it {docker container} bash
```

## 📦 Packages

### ROS2 Packages

#### mission_ctrl
Mission control package for autonomous navigation and task execution.

#### opencv_pkg
Computer vision processing using OpenCV for object detection and recognition.

### RealSense Package

The repository includes the Intel RealSense ROS2 wrapper with examples: 

- **Align Depth to Color**:  Align depth stream to color stream
  ```bash
  ros2 launch realsense2_camera rs_align_depth_launch. py
  ```
  or
  ```bash
  ros2 launch realsense2_camera rs_launch.py align_depth. enable: =true
  ```

## 🎯 Usage

### Utilize Docker and ros2 launch to automatically open nodes for the game


```bash
cd docker
# Build and run docker containers (specific instructions TBD)
```

---

**Competition**:  TDK 2025  
**Last Updated**: December 2025
