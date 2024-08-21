#!/usr/bin/env python3
from time import sleep, time
from typing import Literal, Union

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from .common import LatLon, get_2d_distance
from .assignments import Assignment, SurfaceAssignment, WaypointAndDepth
from ..utils.topics import (
    RUDDER_X_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    LOCALIZATION_STATUS,
    DEPTH_CONTROL_CMD,
    PRESSURE_STATUS,
)

from eel_interfaces.action import Navigate
from eel_interfaces.msg import Coordinate, ImuStatus, DepthControlCmd, PressureStatus
from std_msgs.msg import Float32


TARGET_DISTANCE_LIMIT = 2500

UPDATE_FREQUENCY_HZ = 5
SLEEP_TIME = 1 / UPDATE_FREQUENCY_HZ


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__("navigation_action_server", parameter_overrides=[])
        self.logger = self.get_logger()
        self.current_goal: Union[ServerGoalHandle, None] = None

        self._action_server = ActionServer(
            self,
            Navigate,
            "navigate",
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.position_subscription = self.create_subscription(
            Coordinate, LOCALIZATION_STATUS, self.handle_location_update, 10
        )
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_update, 10
        )
        self.pressure_subscription = self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_update, 10
        )
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_X_CMD, 10)
        self.depth_control_publisher = self.create_publisher(
            DepthControlCmd, DEPTH_CONTROL_CMD, 10
        )

        self.current_position: Union[Coordinate, None] = None
        self.distance_to_target = 0.0
        self.bearing_to_target = 0.0
        self.current_heading = 0.0
        self.current_depth = 0.0

        self.logger.info("Navigation action server started.")

    def handle_location_update(self, msg: Coordinate) -> None:
        self.current_position = msg

    def handle_imu_update(self, msg: ImuStatus) -> None:
        self.current_heading = msg.heading

    def handle_pressure_update(self, msg: PressureStatus) -> None:
        self.current_depth = msg.depth

    def publish_rudder_cmd(self, rudder_turn: float) -> None:
        rudder_msg = Float32()
        rudder_msg.data = float(rudder_turn)
        self.rudder_publisher.publish(rudder_msg)

    def publish_depth_cmd(self, target_depth: float) -> None:
        msg = DepthControlCmd()
        msg.depth_target = target_depth
        msg.depth_pid_type = "not important"
        msg.pitch_pid_type = "not important"
        msg.pitch_target = 0.0
        self.depth_control_publisher.publish(msg)

    def publish_motor_cmd(self, motor_value: float) -> None:
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.motor_publisher.publish(motor_msg)

    def handle_accepted_callback(
        self, goal_handle: ServerGoalHandle
    ) -> Literal[GoalResponse.ACCEPT, GoalResponse.REJECT]:
        if goal_handle.is_active:
            print("is active")
        else:
            print("is NOT active")
        if self.current_goal is None:
            self.current_goal = goal_handle
            self.current_goal.execute()
            return GoalResponse.ACCEPT
        else:
            self.logger.info("Goal in progress. Cancel before asking for another goal.")
            return GoalResponse.REJECT

    def goal_callback(
        self, goal_request: Navigate.Goal
    ) -> Literal[GoalResponse.ACCEPT, GoalResponse.REJECT]:
        if self.current_position is None:
            self.logger.info("No gps position has been aquired yet, rejecting goal.")
            return GoalResponse.REJECT

        distance_to_target = get_2d_distance(
            pos1=LatLon(lat=self.current_position.lat, lon=self.current_position.lon),
            pos2=LatLon(
                lat=goal_request.next_coordinate.lat,
                lon=goal_request.next_coordinate.lon,
            ),
        )

        # Simple sanity check that the target is not to far away, could be removed
        if distance_to_target < TARGET_DISTANCE_LIMIT:
            self.logger.info(
                f"Accepted new target, lat: {goal_request.next_coordinate.lat} lon: {goal_request.next_coordinate.lon}"
            )
            self.logger.info(f"Distance to new target: {distance_to_target}m")

            return GoalResponse.ACCEPT
        else:
            self.logger.info(
                f"Goal rejected - Distance to target {distance_to_target} is larget then set limit {TARGET_DISTANCE_LIMIT}"
            )

            return GoalResponse.REJECT

    def cancel_callback(
        self, goal_handle: ServerGoalHandle
    ) -> Literal[CancelResponse.ACCEPT, CancelResponse.REJECT]:
        self.logger.info(f"Goal cancel request received, turning off motors.")
        self.current_goal = None
        self.publish_motor_cmd(0.0)
        self.publish_rudder_cmd(0.0)

        return CancelResponse.ACCEPT

    def create_assignment(self, goal_request: Navigate.Goal) -> Assignment:
        if goal_request.type == Navigate.Goal.TYPE_WAYPOINT:
            return WaypointAndDepth(
                target_pos=LatLon(
                    lat=goal_request.next_coordinate.lat,
                    lon=goal_request.next_coordinate.lon,
                ),
                depth=(
                    0.0
                    if len(goal_request.next_coordinate_depth) == 0
                    else goal_request.next_coordinate_depth[0]
                ),
                on_set_motor=self.publish_motor_cmd,
                on_set_rudder=self.publish_rudder_cmd,
                on_set_depth=self.publish_depth_cmd,
            )
        elif goal_request.type == Navigate.Goal.TYPE_SURFACE_SYNC:
            return SurfaceAssignment(
                target_pos=LatLon(
                    lat=goal_request.next_coordinate.lat,
                    lon=goal_request.next_coordinate.lon,
                ),
                depth=0.0,
                on_set_motor=self.publish_motor_cmd,
                on_set_rudder=self.publish_rudder_cmd,
                on_set_depth=self.publish_depth_cmd,
            )
        raise TypeError("Unrecognized type for navigate goal")

    def send_feedback(self, distance_to_target: float) -> None:
        feedback_msg = Navigate.Feedback()
        feedback_msg.distance_to_target = distance_to_target
        self.goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Navigate.Result:
        if self.current_position is None:
            raise TypeError("No current position. Cannot do calculations.")

        self.goal_handle = goal_handle

        goal_request: Navigate.Goal = goal_handle.request
        self.assignment = self.create_assignment(goal_request)

        next_coordinate = goal_request.next_coordinate
        # TODO: print distance to target here.
        self.logger.info(
            f"Executing new goal, target set to {next_coordinate.lat=} {next_coordinate.lon=}, {self.distance_to_target=}"
        )

        self.assignment.start()

        while not goal_handle.is_cancel_requested and not self.assignment.get_is_done():
            progress = self.assignment.step(
                current_position=LatLon(
                    lat=self.current_position.lat, lon=self.current_position.lon
                ),
                current_heading=self.current_heading,
                current_depth=self.current_depth,
                current_time_seconds=time(),
            )
            goal_handle.publish_feedback(
                Navigate.Feedback(distance_to_target=progress["distance_to_target"])
            )

            sleep(SLEEP_TIME)

        if not goal_handle.is_cancel_requested:
            goal_handle.succeed()
        else:
            goal_handle.canceled()
        self.current_goal = None
        self.logger.info("Finished goal.")
        self.publish_motor_cmd(0.0)
        self.publish_rudder_cmd(0.0)

        result = Navigate.Result()
        result.success = True

        return result


def main(args=None):
    rclpy.init(args=args)

    navigation_action_server = NavigationActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(navigation_action_server, executor=executor)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
