#! /usr/bin/bash

# include the functions library
source ./robot_functions.bash

# robot statistics

# this is an infinite while loop - use ctrl+c to break
echo "Running Robot Statistics with Bash Script..."
echo "Press Ctrl+C to Terminate..."

# main while loop for naive obstacle avoider
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
while :
do
  # print distance covered since start
  echo "distance covered: $(get_odom_distance)"
  # print current direction of robot
  echo "current direction $(get_odom_direction)"
  # print odom position x, y, z
  echo "odom position: x-$(get_odom_position_x) y-$(get_odom_position_y) z-$(get_odom_position_z)"
  # print odom orientation r, p, y
  echo "odom orientation: r-$(get_odom_orientation_r) p-$(get_odom_orientation_p) y-$(get_odom_orientation_y)"
  # print imu angular velocity x, y, z
  echo "imu angular velocity: x-$(get_imu_angular_velocity_x) y-$(get_imu_angular_velocity_y) z-$(get_imu_angular_velocity_z)"
  # print imu linear acceleration x, y, z
  echo "imu linear acceleration: x-$(get_imu_linear_acceleration_x) y-$(get_imu_linear_acceleration_y) z-$(get_imu_linear_acceleration_z)"

  # print a divider line to show iteration is complete
  echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
done

# End of Code