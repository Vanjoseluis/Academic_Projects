# 🐢 ROS 2 Obstacle Avoider in Python

This project implements a ROS 2 node written in Python to control a mobile robot that avoids obstacles using laser scan data (`LaserScan`), orientation information (`IMU`/odometry), and movement commands (`Twist`). It is a simple and extensible reactive solution for simulations using TurtleBot-like robots.

## 🚀 Features

- 🚧 Obstacle avoidance based on frontal sectors (left, center, right).
- 🧭 Orientation tracking (`yaw`) to determine the robot's cardinal direction.
- 🔄 Reactive algorithm: makes decisions to turn or move forward based on distance readings.
- ✅ Easily extendable logic for navigation, mapping, or behavior trees.

## ⚙️ Requirements

- ROS 2 Humble (or compatible distribution)
- Python 3.10+
- Required packages:
  - `rclpy`
  - `sensor_msgs`
  - `geometry_msgs`
  - `nav_msgs`

---

## 🎥 Audiovisual content
### Real Robot Demo
* I recommend you to visit [my video](https://youtu.be/6Ff-6WbN0mU?si=IfbdimruFISSB1Te) showing the Real Robot on live and some code.
[![Obstacle Avoider Demo](https://img.youtube.com/vi/6Ff-6WbN0mU/0.jpg)](https://youtu.be/6Ff-6WbN0mU)

## 🎤 Project Presentation
* Also my presentation of the project on *The Construct* [**YouTube**](https://www.youtube.com/live/sTN2b3UEYlE?si=bRWYzU6NnODBstbc) channel to see how it works
[![The Construct Presentation](https://img.youtube.com/vi/sTN2b3UEYlE/0.jpg)](https://www.youtube.com/live/sTN2b3UEYlE)

#
This project is licensed under the MIT License. See the LICENSE file for details.
Made with ❤️ for educational robotics.
