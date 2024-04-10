from typing import Optional

from rclpy.node import Node

from moveit.planning import MoveItPy

from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.planning_scene import PlanningScene
from moveit.core.controller_manager import ExecutionStatus
from moveit.core.robot_trajectory import RobotTrajectory

from moveit.planning import PlanningComponent, PlanningSceneMonitor, PlanRequestParameters, TrajectoryExecutionManager

from moveit_msgs.msg import MoveItErrorCodes, AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import GetCartesianPath

from geometry_msgs.msg import Pose, PoseStamped


class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface')
        
        # Initialize MoveItPy
        self._ariac_robots = MoveItPy(node_name="ur_moveit_py")

        # Initialize planning components
        self._robot: PlanningComponent = self._ariac_robots.get_planning_component("ur_manipulator")
        
        # Initialize planning scene monitor
        self._planning_scene_monitor: PlanningSceneMonitor = self._ariac_robots.get_planning_scene_monitor()

        # Initialize trajectory execution manager
        self._trajectory_execution_manager: TrajectoryExecutionManager = self._ariac_robots.get_trajactory_execution_manager()
        
        # Services
        self._compute_cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")