#!/usr/bin/env python3

import rclpy

from pvrams.robot_interface import RobotInterface

def main(args=None):
    rclpy.init(args=args)

    robot = RobotInterface()
    
    rclpy.spin(robot)

if __name__ == '__main__':
    main()