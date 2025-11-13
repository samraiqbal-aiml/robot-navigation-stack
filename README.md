# 🤖 Robot Navigation Stack

[![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

A ROS2-based autonomous robot navigation stack featuring path planning, obstacle avoidance, and goal-oriented navigation. Built for educational purposes and robotics research.

## ✨ Features

- **Autonomous Navigation**: Goal-oriented path planning and execution
- **Obstacle Awareness**: Basic obstacle detection and avoidance logic
- **ROS2 Integration**: Full ROS2 node architecture with topics and parameters
- **Modular Design**: Separated navigation and path planning components
- **Simulation Ready**: Compatible with Gazebo and other ROS2 simulators

## 🎯 Demo

*[Add screenshots/videos of your navigation system working here]*

## 🛠️ Technologies Used

- **ROS2 Humble**: Robotics middleware
- **Python**: Core programming language
- **Gazebo**: Robot simulation environment
- **Navigation2**: ROS2 navigation framework (compatible)
- **Geometry Messages**: ROS2 standard message types

## 📦 Installation

### Prerequisites
- ROS2 Humble (or newer)
- Ubuntu 22.04+ (recommended)
- Python 3.8+

### Setup
```bash
# Clone the repository
git clone https://github.com/samraiqbal-aiml/robot-navigation-stack.git
cd robot-navigation-stack

# Build the package
colcon build
source install/setup.bash
