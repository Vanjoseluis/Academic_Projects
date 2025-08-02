#!/usr/bin/python3

# python imports
import time
import threading
import traceback
import math
# ros2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
# module imports
from robot_interface import RobotInterface
from statistics import multimode



#~#~#~#~#~# start your class after this line #~#~#~#~#~#

class RobotControl:

    def __init__(self, robot_interface):
        self.robot = robot_interface
        return None
    
    def __del__(self):
        # class destructor
        # write your termination codes here if any
        return None
    
    #~#~#~#~#~# add your functions after this line #~#~#~#~#~#
    # Movement functions
    def get_linear_angular_velocities(self):
        return [self.robot.linear_velocity, self.robot.angular_velocity]

    def stop(self):
        self.robot.linear_velocity = 0.0
        self.robot.angular_velocity = 0.0
        return None

    def move_forward(self, linear = 0.1):
        self.robot.linear_velocity = linear
        return None
    
    def move_straight_forward(self, linear = 0.1):
        self.robot.linear_velocity = linear
        self.robot.angular_velocity = 0.0
        return None

    def move_backward(self, linear = 0.1):
        self.robot.linear_velocity = -linear
        return None

    def turn_left(self, angular = 0.3):
        self.robot.angular_velocity = angular
        return None

    def turn_right(self, angular = 0.3):
        self.robot.angular_velocity = -angular
        return None

    def timed_move_forward(self, linear, t):
        self.move_forward(linear)
        time.sleep(t)
        self.stop() #dont use stop() if combined movements are wanted
        return None

    def timed_move_backward(self, linear, t):
        self.move_backward(linear)
        time.sleep(t)
        self.stop()
        return None

    def timed_turn_left(self, t, angular = 0.3):
        self.turn_left(angular)
        time.sleep(t)
        self.stop()
        return None

    def timed_turn_right(self, t, angular = 0.3):
        self.turn_right(angular)
        time.sleep(t)
        self.stop()
        return None

    def turn_left_or_right(self, left, right):
        if left > right:
            self.turn_left(0.5)
            self.move_forward(0.02)
        else:
            self.turn_right(0.5)
            self.move_forward(0.02)
        time.sleep(3) #some problems with small obstacles
        

    def move_distance_forward(self, linear, distance):
        t = distance/linear
        self.move_forward(linear)
        time.sleep(t)
        self.stop() 
        return None

    def move_distance_backward(self, linear, distance):
        t = distance/linear
        self.move_backward(linear)
        time.sleep(t)
        self.stop()
        return None

    def turn_angle_left(self, angular, angle):
        t = angle/angular
        self.turn_left(angular)
        time.sleep(t)
        self.stop() 
        return None

    def turn_angle_right(self, angular, angle):
        t = angle/angular
        self.turn_right(angular)
        time.sleep(t)
        self.stop() 
        return None

#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
    #Laser Scanner Functions
    def get_minimun_scan_angle(self):
        return self.robot.scan_angle_min

    def get_maximum_scan_angle(self):
        return self.robot.scan_angle_max

    def get_angle_increment(self):
        return self.robot.scan_angle_increment

    def get_minimum_scan_range (self):
        return self.robot.scan_range_min

    def get_maximum_scan_range (self):
        return self.robot.scan_range_max

    def get_all_scan_ranges (self):
        return self.robot.scan_ranges

    def get_scan_range_index(self, index):
        return self.robot.scan_ranges[index]

    def get_front_scan_range(self):
        return self.get_scan_range_index(2*len(self.robot.scan_ranges)//4)

    def get_back_scan_range(self):
        return self.get_scan_range_index(0)

    def get_left_scan_range(self):
        return self.get_scan_range_index(3*len(self.robot.scan_ranges)//4)

    def get_right_scan_range(self):
        return self.get_scan_range_index(len(self.robot.scan_ranges)//4)

    def get_minimum_range_index_inf(self):     
        ranges = self.get_all_scan_ranges()
        index = -1 #-1 as return parameter denotates all values are inf 
        minimum = math.inf
    
        for i, x in enumerate(ranges):
            if x < minimum and x != math.inf and x != math.nan:
                minimum = x
                index = i

        return [minimum, index]

    def get_maximum_range_index_inf(self):
        ranges = self.get_all_scan_ranges()
        index = -1 #-1 as return parameter denotates all values are 0
        maximum = 0
    
        for i, x in enumerate(ranges):
            if x > maximum and x != math.inf:
                maximum = x
                index = i

        return [maximum, index]
        
#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
    #Odometry
    def get_current_position(self):
        return [self.robot.odom_position_x, 
                self.robot.odom_position_y,
                self.robot.odom_position_z
        ]

    def get_current_orientation(self):
        return [self.robot.odom_orientation_r, 
                self.robot.odom_orientation_p,
                self.robot.odom_orientation_y
        ]

    def get_distance_euclidean(self, x1, y1, x2, y2):
        distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        return distance

#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
    #IMU
    def get_current_angular_velocity(self):
        return [self.robot.imu_angular_velocity_x, 
                self.robot.imu_angular_velocity_y, 
                self.robot.imu_angular_velocity_z
        ]

    def get_current_linear_acceleration(self):
        return [self.robot.imu_linear_acceleration_x, 
                self.robot.imu_linear_acceleration_y,
                self.robot.imu_linear_acceleration_z
        ]

#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
# OBSTACLE PREDICTION
    def obstacle_prediction(self, threshold = 0.3):
        ranges = self.get_all_scan_ranges()

        # Frontal index
        front_index = 2 * len(ranges) // 4 

        # There are [(pi*rad/4)/angle_increment] indexes between frontal and fl, fr
        angle_increment = self.get_angle_increment()
        num_indexes_45deg = int(45*(self.robot.pi / 180) / angle_increment) 

        # Front_left and Front_rigth indexes
        front_right_index = front_index - num_indexes_45deg
        front_left_index = front_index + num_indexes_45deg

        # Frontal 90º of the scan ranges
        frontal_ranges = ranges[front_right_index:front_left_index + 1] #we add 1 to include front_left_index
        # List comprehension (exclude inf values)
        frontal_ranges = [x for x in frontal_ranges if x != math.inf]

        # Minimum value
        min_range = min(frontal_ranges)
        # Threshold comparison 
        if min_range > threshold:
            return "none"
        else:
           # Statistics modes
            modes = multimode(frontal_ranges)
            if len(modes) > 1:
                return "obstacle"
            else:
                return "wall"

#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
# DIRECTION TRACKING
    def direction_tracking(self):
        _, _, yaw = self.get_current_orientation()
        # Normalize
        yaw_shifted = yaw + 2 * self.robot.pi if yaw < 0 else yaw #e.g. -π/2 = 3π/2 = East
        directions = [
            "-N-", "NNW", "N-W", "WNW",
            "-W-", "WSW", "S-W", "SSW",
            "-S-", "SSE", "S-E", "ESE",
            "-E-", "ENE", "N-E", "NNE"
        ]
        # Search sector in yaw
        sector_size = 2 * self.robot.pi/ 16
        index = int(yaw_shifted // sector_size) % len(directions) #because if yaw = 2pi, index = 16 --> out of range
        index = 0 if index == 16 else index
        return directions[index]

#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#~#
# OBSTACLE AVOIDER
    def obstacle_avoider(self, threshold = 0.3):
        # Init variables
        start_time = time.time()
        self.prev_position = self.get_current_position()
        total_distance = 0.0
        last_print_second = -1
        current_second = 0

        # For 300 seconds (5 minutes)
        while time.time() - start_time < 300:
            ranges = self.get_all_scan_ranges() 
            segment_size = len(ranges) // 16 #16 segments, 2 per each one 

            left = min(ranges[11*segment_size:13*segment_size])
            front_left = min(ranges[9*segment_size:11*segment_size])
            front = min(ranges[7*segment_size:9*segment_size])
            front_right = min(ranges[5*segment_size:7*segment_size])
            right = min(ranges[3*segment_size:5*segment_size])

            if front_right > threshold and front > threshold and front_left > threshold:
                self.move_straight_forward()
            else:
                self.stop()
                if front > threshold:
                    if front_left < threshold and front_right > threshold:
                        self.turn_right()
                        self.move_forward()
                    elif front_right < threshold and front_left > threshold:
                        self.turn_left()
                        self.move_forward()
                    else:
                        self.turn_left_or_right(left, right)
                else:
                    self.turn_left_or_right(left, right)

            current_second = int(time.time())
            # Every 5 seconds
            if current_second % 5 == 0 and current_second != last_print_second:
                current_position = self.get_current_position()
                distance = self.get_distance_euclidean(
                    self.prev_position[0], self.prev_position[1],
                    current_position[0], current_position[1]
                )
                total_distance += distance
                self.prev_position = current_position
                last_print_second = current_second

                print("\n")
                print("Obstacle Prediction: ", self.obstacle_prediction())
                print("Directions: ", self.direction_tracking())

                print("Distance covered:", total_distance)
                print("IMU Angular Velocity:", self.get_current_angular_velocity())
                print("IMU Linear Acceleration:", self.get_current_linear_acceleration())
                #time.sleep(0.5)

#~#~#~#~#~# finish your class before this line #~#~#~#~#~#

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
        #~#~#~#~#~# start your program after this line #~#~#~#~#~#
        robot = RobotControl(robot_interface)

        #~#~#~#~#~# write code here to run only once #~#~#~#~#~#
        robot.obstacle_avoider()
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
