#!/usr/bin/env python3
from time import sleep
from typing import Literal, Union

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ..utils.nav import (
    get_distance_in_meters,
    get_relative_bearing_in_degrees,
    get_next_rudder_turn,
)
from ..utils.topics import (
    RUDDER_X_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    LOCALIZATION_STATUS,
    DEPTH_CONTROL_CMD,
)

from eel_interfaces.action import Navigate
from eel_interfaces.msg import Coordinate, ImuStatus, DepthControlCmd
from std_msgs.msg import Float32

TOLERANCE_IN_METERS = 5.0
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
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_X_CMD, 10)
        # self.depth_control_publisher = self.create_publisher(
        #     DepthControlCmd, DEPTH_CONTROL_CMD, 10
        # )

        self.current_position: Union[Coordinate, None] = None
        self.target_position: Union[Navigate.Goal, None] = None
        self.distance_to_target = 0.0
        self.bearing_to_target = 0.0
        self.current_heading = 0.0

        self.logger.info("Navigation action server started.")

    def handle_location_update(self, msg: Coordinate) -> None:
        self.current_position = msg

    def handle_imu_update(self, msg: ImuStatus) -> None:
        self.current_heading = msg.heading

    def publish_rudder_cmd(self, rudder_turn: float) -> None:
        rudder_msg = Float32()
        rudder_msg.data = float(rudder_turn)
        self.rudder_publisher.publish(rudder_msg)

    # def publish_depth_cmd(self, target_depth: float) -> None:
    #     msg = DepthControlCmd()
    #     msg.depth_target = target_depth
    #     msg.depth_pid_type = "not important"
    #     msg.pitch_pid_type = "not important"
    #     msg.pitch_target = 0.0
    #     self.depth_control_publisher.publish(msg)

    def publish_motor_cmd(self, motor_value: float) -> None:
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.motor_publisher.publish(motor_msg)

    def get_own_distance_to_target(
        self, current_position: Coordinate, target: Navigate.Goal
    ) -> float:
        distance_to_target = get_distance_in_meters(
            current_position.lat,
            current_position.lon,
            target.lat,
            target.lon,
        )

        return distance_to_target

    def handle_accepted_callback(
        self, goal_handle: ServerGoalHandle
    ) -> Literal[GoalResponse.ACCEPT, GoalResponse.REJECT]:
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

        distance_to_target = get_distance_in_meters(
            self.current_position.lat,
            self.current_position.lon,
            goal_request.lat,
            goal_request.lon,
        )

        # Simple sanity check that the target is not to far away, could be removed
        if distance_to_target < TARGET_DISTANCE_LIMIT:
            self.logger.info(
                f"Accepted new target, lat: {goal_request.lat} lon: {goal_request.lon}"
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

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Navigate.Result:
        goal_request: Navigate.Goal = goal_handle.request
        self.target_position = goal_request

        if self.current_position is None:
            raise TypeError("No current position. Cannot do calculations.")

        self.logger.info(
            f"Executing new goal, target set to {self.target_position}, distance to target {self.distance_to_target}"
        )

        feedback_msg = Navigate.Feedback()
        result = Navigate.Result()
        result.lat = self.current_position.lat
        result.lon = self.current_position.lon

        # self.publish_depth_cmd(1.0)

        while goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                self.logger.info("Goal canceled")
                goal_handle.abort()
                break

            self.distance_to_target = self.get_own_distance_to_target(
                self.current_position, goal_request
            )

            feedback_msg.distance_to_target = self.distance_to_target
            goal_handle.publish_feedback(feedback_msg)

            if self.distance_to_target <= TOLERANCE_IN_METERS:
                goal_handle.succeed()
                self.current_goal = None
                self.logger.info("Reached goal. Shutting off stuff.")
                self.publish_motor_cmd(0.0)
                self.publish_rudder_cmd(0.0)

            self.bearing_to_target = get_relative_bearing_in_degrees(
                self.current_position.lat,
                self.current_position.lon,
                self.target_position.lat,
                self.target_position.lon,
            )

            next_rudder_turn = get_next_rudder_turn(
                self.current_heading, self.bearing_to_target
            )

            # TODO: this can be moved outside of the while loop, as it was.
            # and the depth can be set outside as well, depending on the nature of the goal.
            self.publish_motor_cmd(1.0)
            self.publish_rudder_cmd(next_rudder_turn)

            sleep(SLEEP_TIME)

        # TODO: remove
        print("GOAL NO LONGER ACTIVE")
        self.publish_motor_cmd(0.0)
        self.publish_rudder_cmd(0.0)

        return result


def main(args=None):
    rclpy.init(args=args)

    navigation_action_server = NavigationActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(navigation_action_server, executor=executor)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
