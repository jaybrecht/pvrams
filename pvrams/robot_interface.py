from typing import Optional

from math import pi

from time import sleep

from copy import deepcopy

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
        
        self._named_states = {
            "home": [0, -pi/2, pi/2, -pi/2, -pi/2, 0],
        }
        
    def move_to_named_state(self, named_state: str, vsf=1.0, asf=1.0):
        if not named_state in self._named_states.keys():
            self.get_logger().warn(f'Unable to plan to named state {named_state}')
        
        with self._planning_scene_monitor.read_write() as scene:
            scene: PlanningScene
            scene.robot_model
            self._robot.set_start_state(robot_state=scene.current_state)

            goal_state: RobotState = scene.current_state
            goal_state.clear_attached_bodies()
            goal_state.set_joint_group_positions("ur_manipulator", self._named_states[named_state])
            
            self._robot.set_goal_state(robot_state=goal_state)
        
        self._plan_and_execute(vsf=vsf, asf=asf)
        
    def move_robot_cartesian(self, waypoints: list[Pose], vsf=1.0, asf=1.0, avoid_collisions=True):
        result = self._plan_cartesian_trajectory("ur_manipulator", waypoints, vsf, asf, avoid_collisions=avoid_collisions)
        
        if result.fraction < 1.0:
            self.get_logger().warn(f'Unable to fully compute cartesian path, fraction is {result.fraction}')
            return
        
        with self._planning_scene_monitor.read_write() as scene:
            scene: PlanningScene
            trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())            
            trajectory.set_robot_trajectory_msg(scene.current_state, result.solution)
            trajectory.joint_model_group_name = 'ur_manipulator'

        # Retime trajectory
        if not trajectory.apply_totg_time_parameterization(vsf, asf):
            self.get_logger().warn('Unable to retime trajectory')

        execution: ExecutionStatus =  self._ariac_robots.execute(trajectory, controllers=[])
        
        if not execution.status == "SUCCEEDED":
            self.get_logger().error(f'Unable to complete trajectory. Error: {execution.status}')
            
    def get_robot_pose(self) -> Pose:
        with self._planning_scene_monitor.read_only() as scene:
            scene: PlanningScene
            current_state: RobotState = scene.current_state
            
            return current_state.get_pose("tool0")
        
    def follow_box(self, width, length):
        center_pose = self.get_robot_pose()
        
        corners = ((-width/2, length/2), (width/2, length/2), (width/2, -length/2), (-width/2, -length/2), (-width/2, length/2))
        
        waypoints = []
        for x_offset, y_offset in corners:
            corner_pose = deepcopy(center_pose)
            corner_pose.position.x += x_offset
            corner_pose.position.y += y_offset
            
            waypoints.append(corner_pose)
            
        self.move_robot_cartesian(waypoints, vsf=0.1, asf=0.1)
        
    def _plan_and_execute(self, vsf=1.0, asf=1.0):
        # Create the plan request parameters 
        single_plan_parameters = PlanRequestParameters(self._ariac_robots, "ur_manipulator")
        single_plan_parameters.max_acceleration_scaling_factor = asf
        single_plan_parameters.max_velocity_scaling_factor = vsf
        single_plan_parameters.planning_pipeline = "ompl"
        single_plan_parameters.planner_id = "RRTConnectkConfigDefault"

        # Generate plan
        plan: MotionPlanResponse = self._robot.plan(single_plan_parameters=single_plan_parameters)
        plan_result: MoveItErrorCodes = plan.error_code

        if plan_result.val == MoveItErrorCodes.SUCCESS:
            if not plan.trajectory.apply_totg_time_parameterization(vsf, asf):
                self.get_logger().warn('Unable to retime trajectory')
            
            execution: ExecutionStatus =  self._ariac_robots.execute(plan.trajectory, controllers=[])
            
            if not execution.status == "SUCCEEDED":
                self.get_logger().error(f'Unable to complete trajectory. Error: {execution.status}')
        else:
            self.get_logger().error(f'Unable to plan trajectory. Error code: {plan_result.val}')
            
            
    def _plan_cartesian_trajectory(self, group_name: str, waypoints: list[Pose], vsf=1.0, asf=1.0, avoid_collisions=True) -> GetCartesianPath.Response:
        request = GetCartesianPath.Request()

        request.header.frame_id = 'world'
        request.header.stamp = self.get_clock().now().to_msg()
        
        with self._planning_scene_monitor.read_write() as scene:
            scene: PlanningScene
            request.start_state = robotStateToRobotStateMsg(scene.current_state)
            
        request.group_name = group_name

        request.waypoints = waypoints
        request.max_step = 0.1
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = vsf
        request.max_acceleration_scaling_factor = asf
        request.avoid_collisions = avoid_collisions

        future = self._compute_cartesian_path_client.call_async(request)

        while not future.done():
            sleep(0.1)

        return future.result() # type: ignore
            
    