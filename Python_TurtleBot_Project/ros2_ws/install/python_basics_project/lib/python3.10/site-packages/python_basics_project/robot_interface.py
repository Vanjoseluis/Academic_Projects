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
import math


# define robot interface class as a subclass of node class
class RobotInterface(Node):
    # class constructor
    def __init__(self):
        super().__init__("robot_interface")
        self.get_logger().info("Initializing Robot Interface ...")

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
    linear_velocity = 0.0
    angular_velocity = 0.0
    # scan subscriber variables
    scan_msg = LaserScan()
    scan_angle_min = 0.0
    scan_angle_max = 0.0
    scan_angle_increment = 0.0
    scan_range_min = 0.0
    scan_range_max = 0.0
    scan_ranges = list()
    # odom subscriber variables
    odom_msg = Odometry()
    odom_position_x = 0.0
    odom_position_y = 0.0
    odom_position_z = 0.0
    odom_orientation_r = 0.0
    odom_orientation_p = 0.0
    odom_orientation_y = 0.0
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
        self.scan_angle_min = scan_msg.angle_min
        self.scan_angle_max = scan_msg.angle_max
        self.scan_angle_increment = scan_msg.angle_increment
        self.scan_range_min = scan_msg.range_min
        self.scan_range_max = scan_msg.range_max
        self.scan_ranges = [round(value, 3) for value in scan_msg.ranges]
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
        # set robot speeds
        self.twist_cmd.linear.x = self.linear_velocity
        self.twist_cmd.angular.z = self.angular_velocity
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


def main(args=None):

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
