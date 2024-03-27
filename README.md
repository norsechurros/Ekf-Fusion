## **ekfFusion**

![GitHub](https://img.shields.io/github/license/yourusername/ekfFusion)
![GitHub last commit](https://img.shields.io/github/last-commit/yourusername/ekfFusion)
![GitHub contributors](https://img.shields.io/github/contributors/yourusername/ekfFusion)
![GitHub issues](https://img.shields.io/github/issues/yourusername/ekfFusion)

### **Overview**

`ekfFusion` is a ROS package designed for sensor fusion using Extended Kalman Filter (EKF). It integrates data from IMU, GPS, and odometry sources to estimate the pose (position and orientation) of a robot or a vehicle. This repository serves as a comprehensive solution for accurate localization and navigation in robotic applications.

### **Features**

- **Sensor Fusion:** Implements Extended Kalman Filter to fuse data from multiple sensors.
- **Supported Sensors:**
  - IMU (Inertial Measurement Unit)
  - GPS (Global Positioning System)
  - Odometry
- **ROS Integration:** Designed to work seamlessly within the Robot Operating System (ROS) environment.
- **VectorNav Integration:** Utilizes VectorNav package for IMU interfacing.
- **UTM Conversion:** Includes scripts for obtaining GPS data and transforming it into UTM (Universal Transverse Mercator) values.

### **Installation**

#### **Prerequisites**

- ROS (Robot Operating System) installed on your system. Follow the [ROS installation instructions](http://wiki.ros.org/ROS/Installation) if you haven't already installed it.
- [robot_localization](http://wiki.ros.org/robot_localization) ROS package.

#### **Building**

Clone the repository into your ROS workspace and build it using `catkin_make`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/ekfFusion.git
cd ..
catkin_make
```

### **Usage**

1. Launch the `ekfFusion` node:

```bash
roslaunch ekfFusion ekf_fusion.launch
```

2. Subscribe to the fused pose topic to obtain the localization information.

### **Configuration**

- **Configuration File:** Adjust the sensor parameters and EKF settings in the configuration file located at `ekfFusion/config/ekf_params.yaml`.

### **Contributing**

Contributions are welcome! If you find any issues or want to suggest improvements, feel free to open an issue or submit a pull request.

### **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### **Acknowledgments**

- Special thanks to the contributors of the `robot_localization` ROS package for providing a robust framework for sensor fusion.
- Credits to the developers of the VectorNav package for seamless IMU interfacing.
