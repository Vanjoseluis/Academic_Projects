#!/usr/bin/python3

# python imports
import time
import threading
import traceback
# ros2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
# module imports
from robot_interface import RobotInterface


#~#~#~#~#~# start your function definitions after this line #~#~#~#~#~#

def sample_move(linear, angular):
    # sample function to move the robot
    # this function can be deleted later!
    global robot_interface
    robot_interface.linear_velocity = linear
    robot_interface.angular_velocity = angular
    return None

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
        #~#~#~#~#~# start your program after this line #~#~#~#~#~#

        #~#~#~#~#~# write code here to run only once #~#~#~#~#~#
        sample_move(0.500, 0.000)
        time.sleep(1.0)
        sample_move(0.000, 0.000)
        time.sleep(1.0)

        #~#~#~#~#~# write code here to run continuously #~#~#~#~#~#
        while True:
            sample_move(0.100, 0.000)
            time.sleep(1.0)
            sample_move(0.000, 0.000)
            time.sleep(1.0)
        
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
