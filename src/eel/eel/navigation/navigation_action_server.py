#!/usr/bin/env python3
import collections
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
)

from eel_interfaces.action import Navigate
from eel_interfaces.msg import GnssStatus, ImuStatus
from std_msgs.msg import Float32


TOLERANCE_IN_METERS = 5.0
TARGET_DISTANCE_LIMIT = 2500


class NavigationActionServer(Node):

    def __init__(self):
        super().__init__("navigation_action_server", parameter_overrides=[])
        self.logger = self.get_logger()
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

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
            GnssStatus, LOCALIZATION_STATUS, self.handle_gnss_update, 10
        )
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_update, 10
        )
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_X_CMD, 10)

        self.current_position = {"lat": None, "lon": None}
        self.target_position = {"lat": None, "lon": None}
        self.new_position = False
        self.distance_to_target = 0.0
        self.bearing_to_target = 0.0
        self.current_heading = 0.0

        self.logger.info("Navigation action server started.")

    def handle_gnss_update(self, msg):
        self.new_position = True
        self.current_position = {"lat": msg.lat, "lon": msg.lon}

    def handle_imu_update(self, msg):
        self.current_heading = msg.heading

    def publish_rudder_cmd(self, rudder_turn):
        rudder_msg = Float32()
        rudder_msg.data = float(rudder_turn)
        self.rudder_publisher.publish(rudder_msg)

    def publish_motor_cmd(self, motor_value):
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.motor_publisher.publish(motor_msg)

    def get_own_distance_to_target(self):
        distance_to_target = get_distance_in_meters(
            self.current_position["lat"],
            self.current_position["lon"],
            self.target_position["lat"],
            self.target_position["lon"],
        )

        return distance_to_target

    def handle_accepted_callback(self, goal_handle):
        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.logger.info(
                    f"New goal put in queue, queue length: {len(self._goal_queue)}"
                )
            else:
                # Start execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def goal_callback(self, goal_request):
        position_aquired = all(
            [self.current_position.get("lat"), self.current_position.get("lat")]
        )

        if not position_aquired:
            self.logger.info("No gps position has been aquired yet, rejecting goal.")
            return GoalResponse.REJECT

        distance_to_target = get_distance_in_meters(
            self.current_position["lat"],
            self.current_position["lon"],
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

    def cancel_callback(self, goal_handle):
        self.logger.info(f"Goal cancel request sent, turning of motors.")
        self.publish_motor_cmd(0.0)

        with self._goal_queue_lock:
            self.logger.info("Clearing goal queue")
            self._goal_queue.clear()

        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        try:
            self.target_position["lat"] = goal_handle.request.lat
            self.target_position["lon"] = goal_handle.request.lon
            self.distance_to_target = self.get_own_distance_to_target()

            self.logger.info(
                f"Executing new goal, target set to {self.target_position}, distance to target {self.distance_to_target}"
            )

            feedback_msg = Navigate.Feedback()
            result = Navigate.Result()

            # Once target is accepted it is time to start motor
            self.publish_motor_cmd(1.0)

            while self.distance_to_target > TOLERANCE_IN_METERS:

                # Handle cancel calls
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.lon = self.current_position["lon"]
                    result.lat = self.current_position["lat"]
                    self.logger.info("Goal canceled")

                    return result

                # If we have a new GPS position there are values to compute else wait
                if self.new_position:
                    self.distance_to_target = self.get_own_distance_to_target()

                    feedback_msg.distance_to_target = self.distance_to_target
                    goal_handle.publish_feedback(feedback_msg)

                    self.bearing_to_target = get_relative_bearing_in_degrees(
                        self.current_position["lat"],
                        self.current_position["lon"],
                        self.target_position["lat"],
                        self.target_position["lon"],
                    )

                    next_rudder_turn = get_next_rudder_turn(
                        self.current_heading, self.bearing_to_target
                    )
                    self.publish_rudder_cmd(next_rudder_turn)

                    self.new_position = False

            goal_handle.succeed()

            # Once target reached update result and turn off motor
            result.lon = self.current_position["lon"]
            result.lat = self.current_position["lat"]

            self.publish_motor_cmd(0.0)

            return result
        finally:
            with self._goal_queue_lock:
                try:
                    self._current_goal = self._goal_queue.popleft()
                    self.logger.info(
                        f"Next goal pulled from the queue, queue length: {len(self._goal_queue)}"
                    )
                    self._current_goal.execute()
                except IndexError:
                    # No goal in queue
                    self._current_goal = None
                    self.logger.info(f"Goal queue is empty no new goal set")


def main(args=None):
    rclpy.init(args=args)

    navigation_action_server = NavigationActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(navigation_action_server, executor=executor)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
