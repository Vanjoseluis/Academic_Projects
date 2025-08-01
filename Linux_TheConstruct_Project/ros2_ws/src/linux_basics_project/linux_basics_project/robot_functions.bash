#! /usr/bin/bash

# functions

# getters
get_cmd_vel_linear() {
 ros2 param get /robot_interface cmd_vel_linear | awk '{print $4}'
}
get_cmd_vel_angular() {
 ros2 param get /robot_interface cmd_vel_angular | awk '{print $4}'
}
get_scan_angle_min() {
 ros2 param get /robot_interface scan_angle_min | awk '{print $4}'
}
get_scan_angle_max() {
 ros2 param get /robot_interface scan_angle_max | awk '{print $4}'
}
get_scan_angle_increment() {
 ros2 param get /robot_interface scan_angle_increment | awk '{print $4}'
}
get_scan_range_min() {
 ros2 param get /robot_interface scan_range_min | awk '{print $4}'
}
get_scan_range_max() {
 ros2 param get /robot_interface scan_range_max | awk '{print $4}'
}
get_scan_ranges_count() {
 ros2 param get /robot_interface scan_ranges_count | awk '{print $4}'
}
get_scan_ranges_array() {
 ros2 param get /robot_interface scan_ranges_array | awk '{print $4}'
}
get_scan_back_ray_index() {
 ros2 param get /robot_interface scan_back_ray_index | awk '{print $4}'
}
get_scan_right_ray_index() {
 ros2 param get /robot_interface scan_right_ray_index | awk '{print $4}'
}
get_scan_front_right_ray_index() {
 ros2 param get /robot_interface scan_front_right_ray_index | awk '{print $4}'
}
get_scan_front_ray_index() {
 ros2 param get /robot_interface scan_front_ray_index | awk '{print $4}'
}
get_scan_front_left_ray_index() {
 ros2 param get /robot_interface scan_front_left_ray_index | awk '{print $4}'
}
get_scan_left_ray_index() {
 ros2 param get /robot_interface scan_left_ray_index | awk '{print $4}'
}
get_scan_ray_range() {
 ros2 param get /robot_interface scan_ray_range | awk '{print $4}'
}
get_scan_back_ray_range() {
 ros2 param get /robot_interface scan_back_ray_range | awk '{print $4}'
}
get_scan_right_ray_range() {
 ros2 param get /robot_interface scan_right_ray_range | awk '{print $4}'
}
get_scan_front_right_ray_range() {
 ros2 param get /robot_interface scan_front_right_ray_range | awk '{print $4}'
}
get_scan_front_ray_range() {
 ros2 param get /robot_interface scan_front_ray_range | awk '{print $4}'
}
get_scan_front_left_ray_range() {
 ros2 param get /robot_interface scan_front_left_ray_range | awk '{print $4}'
}
get_scan_left_ray_range() {
 ros2 param get /robot_interface scan_left_ray_range | awk '{print $4}'
}
get_odom_position_x() {
 ros2 param get /robot_interface odom_position_x | awk '{print $4}'
}
get_odom_position_y() {
 ros2 param get /robot_interface odom_position_y | awk '{print $4}'
}
get_odom_position_z() {
 ros2 param get /robot_interface odom_position_z | awk '{print $4}'
}
get_odom_distance() {
 ros2 param get /robot_interface odom_distance | awk '{print $4}'
}
get_odom_orientation_r() {
 ros2 param get /robot_interface odom_orientation_r | awk '{print $4}'
}
get_odom_orientation_p() {
 ros2 param get /robot_interface odom_orientation_p | awk '{print $4}'
}
get_odom_orientation_y() {
 ros2 param get /robot_interface odom_orientation_y | awk '{print $4}'
}
get_odom_direction() {
 ros2 param get /robot_interface odom_direction | awk '{print $4}'
}
get_imu_angular_velocity_x() {
 ros2 param get /robot_interface imu_angular_velocity_x | awk '{print $4}'
}
get_imu_angular_velocity_y() {
 ros2 param get /robot_interface imu_angular_velocity_y | awk '{print $4}'
}
get_imu_angular_velocity_z() {
 ros2 param get /robot_interface imu_angular_velocity_z | awk '{print $4}'
}
get_imu_linear_acceleration_x() {
 ros2 param get /robot_interface imu_linear_acceleration_x | awk '{print $4}'
}
get_imu_linear_acceleration_y() {
 ros2 param get /robot_interface imu_linear_acceleration_y | awk '{print $4}'
}
get_imu_linear_acceleration_z() {
 ros2 param get /robot_interface imu_linear_acceleration_z | awk '{print $4}'
}


# setters
set_cmd_vel_linear () {
 ros2 param set /robot_interface cmd_vel_linear "$1"
 return 0
}
set_cmd_vel_angular () {
 ros2 param set /robot_interface cmd_vel_angular "$1"
 return 0
}
set_scan_ray_index () {
 ros2 param set /robot_interface scan_ray_index "$1"
 return 0
}

# End of Code
