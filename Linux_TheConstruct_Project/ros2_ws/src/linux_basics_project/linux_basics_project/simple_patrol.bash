#! /usr/bin/bash

# functions

# getters
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

# setters
set_cmd_vel_linear () {
 ros2 param set /robot_interface cmd_vel_linear "$1"
 return 0
}
set_cmd_vel_angular () {
 ros2 param set /robot_interface cmd_vel_angular "$1"
 return 0
}

# simple robot patrol

# this is an infinite while loop - use ctrl+c to break
echo "Running Simple Robot Patrol with Bash Script..."
echo "Press Ctrl+C to Terminate..."

# make sure that the robot is stopped initially
set_cmd_vel_linear 0.000
set_cmd_vel_angular 0.000

# set obstacle distance threshold
threshold=0.300

# main while loop for simple robot patrol
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
while :
do
  # get the left, front, right scan range values
  ff_range=$(get_scan_front_ray_range)
  echo "front range: $ff_range meters"
  # check if front range is under 0.300 meters
  front_free=$(echo "$ff_range > $threshold" | bc -l)
  echo "front_free: $front_free"
  # if front range is less than threshold
  if [[ "$front_free" == "0" ]]; then
    # check which direction to turn
    ll_range=$(get_scan_left_ray_range)
    echo "left range: $ll_range meters"
    rr_range=$(get_scan_right_ray_range)
    echo "right range: $rr_range meters"
    if (( $(echo "$ll_range > $rr_range" | bc -l ) )); then
      # turn left for roughly 90 degrees
      echo "turning left..."
      set_cmd_vel_angular 0.31416
    else
      # turn right for roughly 90 degrees
      echo "turning right..."
      set_cmd_vel_angular -0.31416
    fi
    # wait for the robot to turn - tune delay accordingly
    sleep 1.500
    # set angular velocity back to zero
    set_cmd_vel_angular 0.000
  else
    # if front range is more than threshold
    # move forward for roughly (front range - threshold) meters
    dist_to_move=$(echo "$ff_range - $threshold" | bc -l)
    # calculate time with fixed speed of 0.1 m/s
    time_to_move=$(echo "$dist_to_move / 0.100" | bc -l)
    # subtract 1 second for parameter setting delay
    time_to_move=$(echo "$time_to_move - 1.000" | bc -l)
    echo "dist_to_move: $dist_to_move meters"
    echo "time_to_move: $time_to_move seconds"
    echo "moving forward..."
    set_cmd_vel_linear 0.100
    sleep $time_to_move
    # set linear velocity back to zero
    set_cmd_vel_linear 0.000
  fi
  # print a divider line to show iteration is complete
  echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
done

# End of Code
