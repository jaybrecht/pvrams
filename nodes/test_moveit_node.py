#!/usr/bin/env python3

import threading


from time import sleep

import rclpy

from rclpy.executors import SingleThreadedExecutor

from pvrams.robot_interface import RobotInterface

def main(args=None):
    rclpy.init(args=args)

    robot = RobotInterface()
    
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    robot.move_to_named_state("home", vsf=0.1, asf=0.1)
    
    sleep(2)
    
    robot.follow_box(0.3, 0.2)

if __name__ == '__main__':
    main()