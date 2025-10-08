# turtlebot3_color_detection_cpp

## Table Of Contents

* [Overview](#-overview)
* [Structure](#-structure)
* [Requirements](#-requirements)
* [Parameters](#-parameters)
* [Usage](#-usage)
* [Results](#-results)
* [Author](#author)

## 🔍 Overview

This repository contains a ROS application that uses C++ and OpenCV to detect a specific color in the scene and move the TurtleBot3 Waffle towards the color target.

> [!NOTE]
>This is one of the exercises I use to introduce Computer Vision with ROS to students.

 ## 📁 Structure

- **launch/**
  - `color_detection.launch`: launches Gazebo simulation with TurtleBot3 and the custom world.
- **src/**
  - `main.cpp`: main code for launching the node and running the program.
  - `camera_subscriber.cpp`: handles the camera subscription and color detection pipeline. 
  - `velocity_publisher.cpp`: defines the movement of the robot and publishes velocities.
- **include/**
  - **color_detection_cpp/**: 
    - `camera_subscriber.hpp`: class definition for camera and color detection operations. 
    - `velocity_publisher.hpp`: class definition for velocity operations.
- **worlds/**
  - `color_detection.world`: custom world with cylinders of different colors.
- `CMakeLists.txt`
- `README.md`
- `package.xml`


## 💻 Requirements
- ROS and turtlebot3 (in this case Noetic was used)
  ```
  sudo apt-get install ros-noetic-gazebo-ros ros-noetic-turtlebot3*
  ```
- C++
- Gazebo
- OpenCV

## 🔧 Parameters

All the parameters are defined within the code. 
You can change the `color` to detect, adjust the `pixel ranges` used for color recognition, and configure different `velocities` to observe how the robot’s movement behavior changes.

## 🚀 Usage

Launch the Gazebo simulation with turtlebot3:
```bash
roslaunch color_detection color_detection.launch
```

After that, in a new cmd tab, you can run the code using:
```bash
rosrun color_detection_cpp color_detection_cpp_node 
```

## 📊 Results
As a result, the robot will rotate to identify the color specified in the code using its camera. Once the color is detected, it will move toward the cylinder. If the robot gets too close to the object due to higher speeds, it will automatically go backwards.

![color_detection_cpp_gif](https://github.com/user-attachments/assets/2fc3d8b7-de9b-403e-b979-95cec87c218e)

## Author

**Esther Vera Moreno**
 * [Personal Github](https://github.com/EstherRobotics)
 * [ICAERUS Github](https://github.com/ICAERUS-EU/UC1_Crop_Monitoring)
 * [Medium Projects](https://medium.com/@esthervera99)
 * [LinkedIn](https://www.linkedin.com/in/estherverarobotics/) 



