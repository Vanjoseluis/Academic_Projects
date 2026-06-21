# 🤖 Bash-Based Naive Obstacle Avoider for ROS 2

This project implements a basic obstacle avoidance system for a mobile robot using Bash scripting and ROS 2 parameters. It uses laser scan (`/scan`), odometry (`/odom`), and IMU data to control the robot's motion via `cmd_vel` parameters.

Designed for simplicity and education, this is a lightweight alternative to Python/Cpp nodes in ROS 2.

## 🧠 Features

- 🛑 Naive obstacle avoidance logic using front, left, and right distance thresholds.
- 🔄 Real-time decision-making via infinite `while` loops in Bash.
- 📈 Live robot telemetry printing (position, orientation, IMU, velocity).
- 🛠 Simple and modular design using `robot_functions.bash` for parameter access.

## ⚙️ Requirements

- ROS 2 Humble (or compatible)
- A robot simulation or hardware exposing:
  - `/robot_interface` node with parameters like `cmd_vel_linear`, `scan_front_ray_range`, etc.
  - `ros2 param` CLI interface
- `bc` installed (for float comparison in Bash)

---

## 🎥 Audiovisual content
### Project Presentation (The Construct)
* I recommend you to visit my presentation of the project on *The Construct* [**YouTube**](https://www.youtube.com/live/my1uggnVLkg?si=Qn4Eo9icft3ClaFf) channel to see how it works
[![Project Presentation](https://img.youtube.com/vi/my1uggnVLkg/0.jpg)](https://www.youtube.com/live/my1uggnVLkg)


#
This project is licensed under the MIT License. See the LICENSE file for details.
Made with ❤️ for educational robotics.
