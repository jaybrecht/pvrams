from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):    
    # Robot Bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_robot_driver"), "/launch", "/ur_control.launch.py"]
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': '192.168.0.1',
            'launch_rviz': 'false'
        }.items()
    )

    nodes_to_start = [
        robot_bringup,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])