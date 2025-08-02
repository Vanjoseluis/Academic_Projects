#!/usr/bin/python3

# python imports
import time
import math
import threading
import traceback
# ros2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
# module imports
from robot_interface import RobotInterface



#~#~#~#~#~# start your function definitions after this line #~#~#~#~#~#

#Robot Movement Functions
def sample_move(linear, angular):
    # sample function to move the robot
    # this function can be deleted later!
    robot_interface.linear_velocity = linear
    robot_interface.angular_velocity = angular
    return None

def get_linear_angular_velocities():
    return [robot_interface.linear_velocity, robot_interface.angular_velocity]

def stop():
    robot_interface.linear_velocity = 0.0
    robot_interface.angular_velocity = 0.0
    return None

def move_forward(linear):
    robot_interface.linear_velocity = linear
    return None

def move_backward(linear):
    robot_interface.linear_velocity = -linear
    return None

def turn_left(angular):
    robot_interface.angular_velocity = angular
    return None

def turn_right(angular):
    robot_interface.angular_velocity = -angular
    return None

def timed_move_forward(linear, t):
    move_forward(linear)
    time.sleep(t)
    stop() #dont use stop() if combined movementsa are wanted
    return None

def timed_move_backward(linear, t):
    move_backward(linear)
    time.sleep(t)
    stop()
    return None

def timed_turn_left(angular, t):
    turn_left(angular)
    time.sleep(t)
    stop()
    return None

def timed_turn_right(angular, t):
    turn_right(angular)
    time.sleep(t)
    stop()
    return None

def move_distance_forward(linear, distance):
    t = distance/linear
    move_forward(linear)
    time.sleep(t)
    stop() 
    return None

def move_distance_backward(linear, distance):
    t = distance/linear
    move_backward(linear)
    time.sleep(t)
    stop()
    return None

def turn_angle_left(angular, angle):
    t = angle/angular
    turn_left(angular)
    time.sleep(t)
    stop() 
    return None

def turn_angle_right(angular, angle):
    t = angle/angular
    turn_right(angular)
    time.sleep(t)
    stop() 
    return None

#Laser Scanner Functions
def get_minimun_scan_angle():
    return robot_interface.scan_angle_min

def get_maximum_scan_angle():
    return robot_interface.scan_angle_max

def get_angle_increment():
    return robot_interface.scan_angle_increment

def get_minimum_scan_range ():
    return robot_interface.scan_range_min

def get_maximum_scan_range ():
    return robot_interface.scan_range_max

def get_all_scan_ranges ():
    return robot_interface.scan_ranges

def get_scan_range_index(index):
    return robot_interface.scan_ranges[index]

def get_front_scan_range():
    return get_scan_range_index(2*len(robot_interface.scan_ranges)//4)

def get_back_scan_range():
    return get_scan_range_index(0)

def get_left_scan_range():
    return get_scan_range_index(3*len(robot_interface.scan_ranges)//4)

def get_right_scan_range():
    return get_scan_range_index(len(robot_interface.scan_ranges)//4)

def get_minimum_range_index_inf():     
        ranges = get_all_scan_ranges()
        index = -1 #-1 as return parameter denotates all values are inf 
        minimum = math.inf
    
        for i, x in enumerate(ranges):
            if x < minimum and x != math.inf and x != math.nan:
                minimum = x
                index = i

        return [minimum, index]

def get_maximum_range_index_inf():
    ranges = get_all_scan_ranges()
    index = -1 #-1 as return parameter denotates all values are 0
    maximum = 0
    
    for i, x in enumerate(ranges):
        if x > maximum and x != math.inf:
            maximum = x
            index = i

    return [maximum, index]


#Odometry
def get_current_position():
    return [robot_interface.odom_position_x, robot_interface.odom_position_y, robot_interface.odom_position_z]

def get_current_orientation():
    return [robot_interface.odom_orientation_r, robot_interface.odom_orientation_p, robot_interface.odom_orientation_y]

def get_distance_euclidean(x1, y1, x2, y2):
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    return distance

#IMU
def get_current_angular_velocity():
    return [robot_interface.imu_angular_velocity_x, robot_interface.imu_angular_velocity_y, robot_interface.imu_angular_velocity_z]

def get_current_linear_acceleration():
    return [robot_interface.imu_linear_acceleration_x, robot_interface.imu_linear_acceleration_y, robot_interface.imu_linear_acceleration_z]

#~#~#~#~#~# finish your function definitions before this line #~#~#~#~#~#

def spin_node():
    """
    make the robot interface program to run in a separate thread
    NOTE: THE ROBOT WILL NOT WORK IF THIS FUNCTION IS REMOVED !!!
    """
    global executor
    executor.spin()
    return None


if __name__ == "__main__":

    # initialize ros2 with python
    rclpy.init(args=None)
    # instantiate robot interface program module
    robot_interface = RobotInterface()
    # start robot interface program execution
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(robot_interface)
    # run robot interface program in a separate thread
    threading.Thread(target=spin_node).start()
    # wait for a few seconds for program to initialize
    print("Getting Ready in 5 Seconds...")
    time.sleep(5.0)
    print("READY !!!")

    try:
        
        #~#~#~#~#~# write code here to run only once #~#~#~#~#~#
        #~#~#~#~#~# start your program after this line #~#~#~#~#~#

        # move forward 20 cm, 0.1 m/s
        print("Moving forward 20 cm, 0.1 m/s...")
        move_distance_forward(0.1, 0.2)  # 0.2 m = 20 cm
        time.sleep(1.0)

        # turn left 90º (pi/2 rad), 0.5 rad/s
        print("Turning left 90º, 0.5 rad/s...")
        turn_angle_left(0.5, 3.1416 / 2)
        time.sleep(1.0)

        # move backward 30 cm, 0.15 m/s
        print("Moving backward 30 cm, 0.15 m/s...")
        move_distance_backward(0.15, 0.3)
        time.sleep(1.0)

        # turn right 45º (pi/4 rad), 0.5 rad/s
        print("Turning right 45º, 0.5 rad/s...")
        turn_angle_right(0.5, 3.1416 / 4)
        time.sleep(1.0)

        # Odometry
        pos = get_current_position()
        orient = get_current_orientation()
        print("Posición actual (x, y, z): ", pos)
        print("Orientación actual (r, p, y): ", orient)

        # Angular velocity IMU
        ang_vel = get_current_angular_velocity()
        print("Velocidad angular IMU (x, y, z): ", ang_vel)

        # Linear velocity IMU
        lin_acc = get_current_linear_acceleration()
        print("Aceleración lineal IMU (x, y, z): ", lin_acc)

        # Lasers distances
        front = get_front_scan_range()
        left = get_left_scan_range()
        right = get_right_scan_range()
        back = get_back_scan_range()
        print("Frontal distance: ", front)
        print("Left distance: ", left)
        print("Right distance: ", right)
        print("Back distance: ", back)

        # Minimum and maximum ranges
        minimum_range = get_minimum_scan_range()
        maximum_range = get_maximum_scan_angle()
        print("Minimum range: ", minimum_range)
        print("Maximum range: ", maximum_range)

        # Parar el robot al finalizar
        print("Stopping robot...")
        stop()

#~#~#~#~#~# finish your program before this line #~#~#~#~#~#


        #~#~#~#~#~# write code here to run continuously #~#~#~#~#~#
        while True:
            pass
        
        #~#~#~#~#~# finish your program before this line #~#~#~#~#~#

    except Exception as error:
        # report exception
        print("~~~~~~~~~~~ ERROR: ~~~~~~~~~~~")
        print(traceback.print_exception(error))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # clean up before shutdown
        executor.shutdown()
        robot_interface.destroy_node()

    finally:
        # shutdown ros2
        rclpy.shutdown()

# End of Code