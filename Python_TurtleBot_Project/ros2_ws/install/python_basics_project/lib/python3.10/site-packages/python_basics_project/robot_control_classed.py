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


#~#~#~#~#~# start your class after this line #~#~#~#~#~#

class RobotControl:

    def __init__(self, robot_interface):
        # class constructor
        # write your initialization codes here if any
        return None
    
    def __del__(self):
        # class destructor
        # write your termination codes here if any
        return None
    
    #~#~#~#~#~# add your functions after this line #~#~#~#~#~#

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

        ########## |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~| ##########
        ########## | ! YOU ARE ON YOUR OWN FROM HERE ! | ##########
        ########## |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~| ##########

        #~#~#~#~#~# write code here to run only once #~#~#~#~#~#

        #~#~#~#~#~# write code here to run continuously #~#~#~#~#~#
        while True:
            pass   # replace pass with your program code
        
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
