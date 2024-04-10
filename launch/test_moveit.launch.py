from launch import LaunchDescription
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder   

def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("ur5e", package_name="aprs_ur5e_moveit_config")
        .robot_description(file_path="config/ur5e.urdf")
        .robot_description_semantic(file_path="config/ur5e.srdf")
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(file_path="config/moveitpy_config.yaml")  
        .to_moveit_configs()
    )

    # PVRAMS node
    move_it_test = Node(
        package="pvrams",
        executable="test_moveit_node.py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    nodes_to_start = [
        move_it_test,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])