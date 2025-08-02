#! /usr/bin/python3

# imports
# rclpy imports
import rclpy                                              # rclpy
from rclpy.node import Node                               # base node
from rclpy.qos import QoSProfile                          # qos profile
from rclpy.qos import (HistoryPolicy, ReliabilityPolicy,  # qos policies
                      DurabilityPolicy, LivelinessPolicy) # qos policies
from rclpy.executors import MultiThreadedExecutor         # multithreaded executor
from rclpy.callback_groups import ReentrantCallbackGroup  # reentrant callback group
# ros2 interfaces
from geometry_msgs.msg import Twist   # twist message
from nav_msgs.msg import Odometry     # odometry message
from sensor_msgs.msg import LaserScan # laser scan message
from sensor_msgs.msg import Imu       # imu message
# standard imports
import os
import math
import time
import statistics


# define robot interface class as a subclass of node class
class RobotInterface(Node):
    # class constructor
    def __init__(self):
        super().__init__("robot_interface", allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("Initializing Robot Interface ...")

        # declare ros2 node parameters along with default values
        # command velocity parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_linear", 0.000),
                ("cmd_vel_angular", 0.000),
            ]
        )
        # laserscan parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scan_angle_min", 0.000),
                ("scan_angle_max", 0.000),
                ("scan_angle_increment", 0.000),
                ("scan_range_min", 0.000),
                ("scan_range_max", 0.000),
                ("scan_ranges_count", 0),
                ("scan_ranges_array", [0.000,]),
                ("scan_ray_index", 0),
                ("scan_back_ray_index", 0),
                ("scan_right_ray_index", 0),
                ("scan_front_right_ray_index", 0),
                ("scan_front_ray_index", 0),
                ("scan_front_left_ray_index", 0),
                ("scan_left_ray_index", 0),
                ("scan_ray_range", 0.000),
                ("scan_back_ray_range", 0.000),
                ("scan_right_ray_range", 0.000),
                ("scan_front_right_ray_range", 0.000),
                ("scan_front_ray_range", 0.000),
                ("scan_front_left_ray_range", 0.000),
                ("scan_left_ray_range", 0.000),
            ]
        )
        # odometry parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("odom_position_x", 0.000),
                ("odom_position_y", 0.000),
                ("odom_position_z", 0.000),
                ("odom_distance", 0.000),
                ("odom_orientation_r", 0.000),
                ("odom_orientation_p", 0.000),
                ("odom_orientation_y", 0.000),
                ("odom_direction", "-N-"),
            ]
        )
        # imu parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("imu_angular_velocity_x", 0.000),
                ("imu_angular_velocity_y", 0.000),
                ("imu_angular_velocity_z", 0.000),
                ("imu_linear_acceleration_x", 0.000),
                ("imu_linear_acceleration_y", 0.000),
                ("imu_linear_acceleration_z", 0.000),
            ]
        )

        # declare and initialize cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,
                                                 topic="/cmd_vel",
                                                 qos_profile=10)
        self.get_logger().info("Initialized Publisher: /cmd_vel")

        # declare and initialize callback group
        self.callback_group = ReentrantCallbackGroup()

        # declare and initialize scan subscriber
        self.scan_sub_qos = QoSProfile(depth=10,
                                       history=HistoryPolicy.KEEP_LAST,
                                       reliability=ReliabilityPolicy.BEST_EFFORT,
                                       durability=DurabilityPolicy.VOLATILE,
                                       liveliness=LivelinessPolicy.AUTOMATIC)
        self.scan_sub = self.create_subscription(msg_type=LaserScan,
                                                 topic="/scan",
                                                 callback=self.scan_callback,
                                                 qos_profile=self.scan_sub_qos,
                                                 callback_group=self.callback_group)
        self.get_logger().info("Initialized Subscriber: /scan")

        # declare and initialize odom subscriber
        self.odom_sub_qos = QoSProfile(depth=10,
                                       history=HistoryPolicy.KEEP_LAST,
                                       reliability=ReliabilityPolicy.BEST_EFFORT,
                                       durability=DurabilityPolicy.VOLATILE,
                                       liveliness=LivelinessPolicy.AUTOMATIC)
        self.odom_sub = self.create_subscription(msg_type=Odometry,
                                                 topic="/odom",
                                                 callback=self.odom_callback,
                                                 qos_profile=self.odom_sub_qos,
                                                 callback_group=self.callback_group)
        self.get_logger().info("Initialized Subscriber: /odom")

        # declare and initialize imu subscriber
        self.imu_sub_qos = QoSProfile(depth=10,
                                      history=HistoryPolicy.KEEP_LAST,
                                      reliability=ReliabilityPolicy.BEST_EFFORT,
                                      durability=DurabilityPolicy.VOLATILE,
                                      liveliness=LivelinessPolicy.AUTOMATIC)
        self.imu_sub = self.create_subscription(msg_type=Imu,
                                                topic="/imu",
                                                callback=self.imu_callback,
                                                qos_profile=self.imu_sub_qos,
                                                callback_group=self.callback_group)
        self.get_logger().info("Initialized Subscriber: /imu")

        # declare and initialize control timer callback
        self.control_timer = self.create_timer(timer_period_sec=0.100,
                                               callback=self.control_callback,
                                               callback_group=self.callback_group)
        self.get_logger().info("Initialized Control Timer")

        self.get_logger().info("Robot Interface Initialized !")

        # wait for subscriber callbacks to initialize properly
        self.get_logger().info("Getting Ready in 5 Seconds...")
        time.sleep(5.0)
        self.get_logger().info("READY !!!")

        return None

    # class destructor
    def __del__(self):
        return None

    # define and initialize class variables
    # general constants
    pi = 3.141592654
    pi_inv = 0.318309886
    # cmd_vel publisher variables
    twist_cmd = Twist()
    cmd_vel_linear = 0.0
    cmd_vel_angular = 0.0
    # scan subscriber variables
    scan_msg = LaserScan()
    scan_angle_min = 0.0
    scan_angle_max = 0.0
    scan_angle_increment = 0.0
    scan_range_min = 0.0
    scan_range_max = 0.0
    scan_ranges_count = 0
    scan_ranges_array = list()
    scan_ray_index = 0
    scan_back_ray_index = 0
    scan_right_ray_index = 0
    scan_front_right_ray_index = 0
    scan_front_ray_index = 0
    scan_front_left_ray_index = 0
    scan_left_ray_index = 0
    scan_ray_range = 0.0
    scan_back_ray_range = 0.0
    scan_right_ray_range = 0.0
    scan_front_right_ray_range = 0.0
    scan_front_ray_range = 0.0
    scan_front_left_ray_range = 0.0
    scan_left_ray_range = 0.0
    # odom subscriber variables
    odom_msg = Odometry()
    odom_init = False
    odom_prev_pos_x = 0.0
    odom_prev_pos_y = 0.0
    odom_prev_pos_z = 0.0
    odom_position_x = 0.0
    odom_position_y = 0.0
    odom_position_z = 0.0
    odom_distance = 0.0
    odom_orientation_r = 0.0
    odom_orientation_p = 0.0
    odom_orientation_y = 0.0
    odom_direction = "-N-"
    # imu subscriber variables
    imu_msg = Imu()
    imu_angular_velocity_x = 0.0
    imu_angular_velocity_y = 0.0
    imu_angular_velocity_z = 0.0
    imu_linear_acceleration_x = 0.0
    imu_linear_acceleration_y = 0.0
    imu_linear_acceleration_z = 0.0

    # private class methods and callbacks

    def scan_callback(self, scan_msg):
        # simple method to get scan data
        self.scan_msg = scan_msg
        self.scan_angle_min = round(scan_msg.angle_min, 3)
        self.scan_angle_max = round(scan_msg.angle_max, 3)
        self.scan_angle_increment = round(scan_msg.angle_increment, 6)
        self.scan_range_min = round(scan_msg.range_min, 3)
        self.scan_range_max = round(scan_msg.range_max, 3)
        self.scan_ranges_count = int(len(scan_msg.ranges))
        self.scan_ranges_array = [round(value, 3) for value in scan_msg.ranges]
        self.scan_back_ray_index = int(self.scan_ranges_count * 0.000)
        self.scan_right_ray_index = int(self.scan_ranges_count * 0.250)
        self.scan_front_right_ray_index = int(self.scan_ranges_count * 0.375)
        self.scan_front_ray_index = int(self.scan_ranges_count * 0.500)
        self.scan_front_left_ray_index = int(self.scan_ranges_count * 0.625)
        self.scan_left_ray_index = int(self.scan_ranges_count * 0.750)
        segments = 8
        rays_per_segment = int(self.scan_ranges_count / segments)
        rays_tolerance = int(rays_per_segment / 2.0)
        self.scan_ray_range = self.scan_ranges_array[self.scan_ray_index]
        self.scan_back_ray_range = min([range_value for range_value in 
            (self.scan_ranges_array[:(self.scan_back_ray_index + rays_tolerance)] +
                self.scan_ranges_array[(self.scan_ranges_count - rays_tolerance):])
            if (not math.isinf(range_value))])
        # self.scan_back_ray_range /= (2 * rays_tolerance)
        self.scan_right_ray_range = min([range_value for range_value in 
            self.scan_ranges_array[(self.scan_right_ray_index - rays_tolerance):
                                    (self.scan_right_ray_index + rays_tolerance)]
            if (not math.isinf(range_value))])
        # self.scan_right_ray_range /= (2 * rays_tolerance)
        self.scan_front_right_ray_range = min([range_value for range_value in 
            self.scan_ranges_array[(self.scan_front_right_ray_index - rays_tolerance):
                                    (self.scan_front_right_ray_index + rays_tolerance)]
            if (not math.isinf(range_value))])
        # self.scan_front_right_ray_range /= (2 * rays_tolerance)
        self.scan_front_ray_range = min([range_value for range_value in 
            self.scan_ranges_array[(self.scan_front_ray_index - rays_tolerance):
                                    (self.scan_front_ray_index + rays_tolerance)]
            if (not math.isinf(range_value))])
        # self.scan_front_ray_range /= (2 * rays_tolerance)
        self.scan_front_left_ray_range = min([range_value for range_value in 
            self.scan_ranges_array[(self.scan_front_left_ray_index - rays_tolerance):
                                    (self.scan_front_left_ray_index + rays_tolerance)]
            if (not math.isinf(range_value))])
        # self.scan_front_left_ray_range /= (2 * rays_tolerance)
        self.scan_left_ray_range = min([range_value for range_value in 
            self.scan_ranges_array[(self.scan_left_ray_index - rays_tolerance):
                                    (self.scan_left_ray_index + rays_tolerance)]
            if (not math.isinf(range_value))])
        # self.scan_left_ray_range /= (2 * rays_tolerance)
        return None

    def odom_callback(self, odom_msg):
        # simple method to get odom data
        self.odom_msg = odom_msg
        position = odom_msg.pose.pose.position
        angles = self.euler_from_quaternion(odom_msg.pose.pose.orientation.x,
                                            odom_msg.pose.pose.orientation.y,
                                            odom_msg.pose.pose.orientation.z,
                                            odom_msg.pose.pose.orientation.w)
        self.odom_position_x = round(position.x, 3)
        self.odom_position_y = round(position.y, 3)
        self.odom_position_z = round(position.z, 3)
        self.odom_orientation_r = round(angles["r"], 3)
        self.odom_orientation_p = round(angles["p"], 3)
        self.odom_orientation_y = round(angles["y"], 3)
        if (self.odom_init):
            self.odom_distance += ((((self.odom_position_x - self.odom_prev_pos_x) ** 2.0) + 
                                    ((self.odom_position_y - self.odom_prev_pos_y) ** 2.0) + 
                                    ((self.odom_position_z - self.odom_prev_pos_z) ** 2.0)) ** 0.500)
            self.odom_distance = round(self.odom_distance, 3)
            self.odom_direction = self.get_direction_from_yaw(yaw=self.odom_orientation_y)
        else:
            self.odom_init = True
        self.odom_prev_pos_x = self.odom_position_x
        self.odom_prev_pos_y = self.odom_position_y
        self.odom_prev_pos_z = self.odom_position_z
        return None
    
    def imu_callback(self, imu_msg):
        # simple method to get imu data
        self.imu_msg = imu_msg
        angular_vel = imu_msg.angular_velocity
        linear_acc = imu_msg.linear_acceleration
        self.imu_angular_velocity_x = round(angular_vel.x, 3)
        self.imu_angular_velocity_y = round(angular_vel.y, 3)
        self.imu_angular_velocity_z = round(angular_vel.z, 3)
        self.imu_linear_acceleration_x = round(linear_acc.x, 3)
        self.imu_linear_acceleration_y = round(linear_acc.y, 3)
        self.imu_linear_acceleration_z = round(linear_acc.z, 3)
        return None

    def control_callback(self):
        # get process variables
        scan_angle_min = rclpy.parameter.Parameter(
            "scan_angle_min",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_angle_min
        )
        scan_angle_max = rclpy.parameter.Parameter(
            "scan_angle_max",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_angle_max
        )
        scan_angle_increment = rclpy.parameter.Parameter(
            "scan_angle_increment",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_angle_increment
        )
        scan_range_min = rclpy.parameter.Parameter(
            "scan_range_min",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_range_min
        )
        scan_range_max = rclpy.parameter.Parameter(
            "scan_range_max",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_range_max
        )
        scan_ranges_count = rclpy.parameter.Parameter(
            "scan_ranges_count",
            rclpy.Parameter.Type.INTEGER,
            self.scan_ranges_count
        )
        scan_ranges_array = rclpy.parameter.Parameter(
            "scan_ranges_array",
            rclpy.Parameter.Type.DOUBLE_ARRAY,
            self.scan_ranges_array
        )
        scan_back_ray_index = rclpy.parameter.Parameter(
            "scan_back_ray_index",
            rclpy.Parameter.Type.INTEGER,
            self.scan_back_ray_index
        )
        scan_right_ray_index = rclpy.parameter.Parameter(
            "scan_right_ray_index",
            rclpy.Parameter.Type.INTEGER,
            self.scan_right_ray_index
        )
        scan_front_right_ray_index = rclpy.parameter.Parameter(
            "scan_front_right_ray_index",
            rclpy.Parameter.Type.INTEGER,
            self.scan_front_right_ray_index
        )
        scan_front_ray_index = rclpy.parameter.Parameter(
            "scan_front_ray_index",
            rclpy.Parameter.Type.INTEGER,
            self.scan_front_ray_index
        )
        scan_front_left_ray_index = rclpy.parameter.Parameter(
            "scan_front_left_ray_index",
            rclpy.Parameter.Type.INTEGER,
            self.scan_front_left_ray_index
        )
        scan_left_ray_index = rclpy.parameter.Parameter(
            "scan_left_ray_index",
            rclpy.Parameter.Type.INTEGER,
            self.scan_left_ray_index
        )
        scan_ray_range = rclpy.parameter.Parameter(
            "scan_ray_range",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_ray_range
        )
        scan_back_ray_range = rclpy.parameter.Parameter(
            "scan_back_ray_range",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_back_ray_range
        )
        scan_right_ray_range = rclpy.parameter.Parameter(
            "scan_right_ray_range",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_right_ray_range
        )
        scan_front_right_ray_range = rclpy.parameter.Parameter(
            "scan_front_right_ray_range",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_front_right_ray_range
        )
        scan_front_ray_range = rclpy.parameter.Parameter(
            "scan_front_ray_range",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_front_ray_range
        )
        scan_front_left_ray_range = rclpy.parameter.Parameter(
            "scan_front_left_ray_range",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_front_left_ray_range
        )
        scan_left_ray_range = rclpy.parameter.Parameter(
            "scan_left_ray_range",
            rclpy.Parameter.Type.DOUBLE,
            self.scan_left_ray_range
        )
        odom_position_x = rclpy.parameter.Parameter(
            "odom_position_x",
            rclpy.Parameter.Type.DOUBLE,
            self.odom_position_x
        )
        odom_position_y = rclpy.parameter.Parameter(
            "odom_position_y",
            rclpy.Parameter.Type.DOUBLE,
            self.odom_position_y
        )
        odom_position_z = rclpy.parameter.Parameter(
            "odom_position_z",
            rclpy.Parameter.Type.DOUBLE,
            self.odom_position_z
        )
        odom_distance = rclpy.parameter.Parameter(
            "odom_distance", 
            rclpy.Parameter.Type.DOUBLE, 
            self.odom_distance
        )
        odom_orientation_r = rclpy.parameter.Parameter(
            "odom_orientation_r",
            rclpy.Parameter.Type.DOUBLE,
            self.odom_orientation_r
        )
        odom_orientation_p = rclpy.parameter.Parameter(
            "odom_orientation_p",
            rclpy.Parameter.Type.DOUBLE,
            self.odom_orientation_p
        )
        odom_orientation_y = rclpy.parameter.Parameter(
            "odom_orientation_y",
            rclpy.Parameter.Type.DOUBLE,
            self.odom_orientation_y
        )
        odom_direction = rclpy.parameter.Parameter(
            "odom_direction", 
            rclpy.Parameter.Type.STRING, 
            self.odom_direction
        )
        imu_angular_velocity_x = rclpy.parameter.Parameter(
            "imu_angular_velocity_x",
            rclpy.Parameter.Type.DOUBLE,
            self.imu_angular_velocity_x
        )
        imu_angular_velocity_y = rclpy.parameter.Parameter(
            "imu_angular_velocity_y",
            rclpy.Parameter.Type.DOUBLE,
            self.imu_angular_velocity_y
        )
        imu_angular_velocity_z = rclpy.parameter.Parameter(
            "imu_angular_velocity_z",
            rclpy.Parameter.Type.DOUBLE,
            self.imu_angular_velocity_z
        )
        imu_linear_acceleration_x = rclpy.parameter.Parameter(
            "imu_linear_acceleration_x",
            rclpy.Parameter.Type.DOUBLE,
            self.imu_linear_acceleration_x
        )
        imu_linear_acceleration_y = rclpy.parameter.Parameter(
            "imu_linear_acceleration_y",
            rclpy.Parameter.Type.DOUBLE,
            self.imu_linear_acceleration_y
        )
        imu_linear_acceleration_z = rclpy.parameter.Parameter(
            "imu_linear_acceleration_z",
            rclpy.Parameter.Type.DOUBLE,
            self.imu_linear_acceleration_z
        )
        # set all process variables as parameters
        parameters = [scan_angle_min, scan_angle_max,
                      scan_angle_increment,
                      scan_range_min, scan_range_max,
                      scan_ranges_count, scan_ranges_array,
                      scan_back_ray_index, scan_right_ray_index, scan_front_right_ray_index,
                      scan_front_ray_index, scan_front_left_ray_index, scan_left_ray_index,
                      scan_ray_range,
                      scan_back_ray_range, scan_right_ray_range, scan_front_right_ray_range,
                      scan_front_ray_range, scan_front_left_ray_range, scan_left_ray_range,
                      odom_position_x, odom_position_y, odom_position_z, odom_distance,
                      odom_orientation_r, odom_orientation_p, odom_orientation_y, odom_direction,
                      imu_angular_velocity_x, imu_angular_velocity_y, imu_angular_velocity_z,
                      imu_linear_acceleration_x, imu_linear_acceleration_y, imu_linear_acceleration_z,
        ]
        self.set_parameters(parameters)
        # set process variables
        self.scan_ray_index = self.get_parameter("scan_ray_index").get_parameter_value().integer_value
        self.cmd_vel_linear = self.get_parameter("cmd_vel_linear").get_parameter_value().double_value
        self.cmd_vel_angular = self.get_parameter("cmd_vel_angular").get_parameter_value().double_value
        # set robot speeds
        self.twist_cmd.linear.x = self.cmd_vel_linear
        self.twist_cmd.angular.z = self.cmd_vel_angular
        # publish the twist command
        self.publish_twist_cmd()
        return None

    def publish_twist_cmd(self):
        # linear speed control
        if (self.twist_cmd.linear.x >= 0.150):
          self.twist_cmd.linear.x = 0.150
        else:
          # do nothing
          pass
        # angular speed control
        if (self.twist_cmd.angular.z >= 0.450):
          self.twist_cmd.angular.z = 0.450
        else:
          # do nothing
          pass
        # publish command
        self.cmd_vel_pub.publish(self.twist_cmd)
        return None

    def euler_from_quaternion(self, quat_x, quat_y, quat_z, quat_w):
        # function to convert quaternions to euler angles
        # calculate roll
        sinr_cosp = 2 * (quat_w * quat_x + quat_y * quat_z)
        cosr_cosp = 1 - 2 * (quat_x * quat_x + quat_y * quat_y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # calculate pitch
        sinp = 2 * (quat_w * quat_y - quat_z * quat_x)
        pitch = math.asin(sinp)
        # calculate yaw
        siny_cosp = 2 * (quat_w * quat_z + quat_x * quat_y)
        cosy_cosp = 1 - 2 * (quat_y * quat_y + quat_z * quat_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # store the angle values in a dict
        angles = dict()
        angles["r"] = roll
        angles["p"] = pitch
        angles["y"] = yaw
        # return the angle values
        return angles

    def get_direction_from_yaw(self, yaw):
        direction = "-N-"
        if (yaw > 0.000):
            if ((yaw > (0 * (math.pi / 16))) and (yaw < (1 * (math.pi / 16)))):
                direction = "-N-"
            elif ((yaw > (1 * (math.pi / 16))) and (yaw < (3 * (math.pi / 16)))):
                direction = "NNW"
            elif ((yaw > (3 * (math.pi / 16))) and (yaw < (5 * (math.pi / 16)))):
                direction = "N-W"
            elif ((yaw > (5 * (math.pi / 16))) and (yaw < (7 * (math.pi / 16)))):
                direction = "WNW"
            elif ((yaw > (7 * (math.pi / 16))) and (yaw < (9 * (math.pi / 16)))):
                direction = "-W-"
            elif ((yaw > (9 * (math.pi / 16))) and (yaw < (11 * (math.pi / 16)))):
                direction = "WSW"
            elif ((yaw > (11 * (math.pi / 16))) and (yaw < (13 * (math.pi / 16)))):
                direction = "S-W"
            elif ((yaw > (13 * (math.pi / 16))) and (yaw < (15 * (math.pi / 16)))):
                direction = "SSW"
            elif ((yaw > (15 * (math.pi / 16))) and (yaw < (16 * (math.pi / 16)))):
                direction = "-S-"
            else:
                pass
        else:
            if ((yaw < (0 * -(math.pi / 16))) and (yaw > (1 * -(math.pi / 16)))):
                direction = "-N-"
            elif ((yaw < (1 * -(math.pi / 16))) and (yaw > (3 * -(math.pi / 16)))):
                direction = "NNE"
            elif ((yaw < (3 * -(math.pi / 16))) and (yaw > (5 * -(math.pi / 16)))):
                direction = "N-E"
            elif ((yaw < (5 * -(math.pi / 16))) and (yaw > (7 * -(math.pi / 16)))):
                direction = "ENE"
            elif ((yaw < (7 * -(math.pi / 16))) and (yaw > (9 * -(math.pi / 16)))):
                direction = "-E-"
            elif ((yaw < (9 * -(math.pi / 16))) and (yaw > (11 * -(math.pi / 16)))):
                direction = "ESE"
            elif ((yaw < (11 * -(math.pi / 16))) and (yaw > (13 * -(math.pi / 16)))):
                direction = "S-E"
            elif ((yaw < (13 * -(math.pi / 16))) and (yaw > (15 * -(math.pi / 16)))):
                direction = "SSE"
            elif ((yaw < (15 * -(math.pi / 16))) and (yaw > (16 * -(math.pi / 16)))):
                direction = "-S-"
            else:
                pass
        return direction


def main(args=None):

    # set environment variable to not show ros2 logger timestamps
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{severity}][{name}]: {message}"

    # initialize ROS2 node
    rclpy.init(args=args)

    # create an instance of the robot interface class
    robot_interface = RobotInterface()
    
    # create a multithreaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    
    # add the robot interface node to the executor
    executor.add_node(robot_interface)

    try:
        # spin the executor to handle callbacks
        executor.spin()
    except:
        pass
    finally:
        # indicate robot interface node termination
        robot_interface.get_logger().info("Terminating Robot Interface ...")
        robot_interface.get_logger().info("Robot Interface Terminated !")
    
    # shutdown the executor when spin completes
    executor.shutdown()
    
    # destroy the robot interface node
    robot_interface.destroy_node()

    # shutdown ROS2 node when spin completes
    rclpy.shutdown()

    return None


if __name__ == "__main__":
    main()

# End of Code
