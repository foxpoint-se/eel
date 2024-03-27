from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from eel_interfaces.action import Navigate
from eel_interfaces.msg import NavigationStatus, Coordinate
from ..utils.topics import (
    NAVIGATION_CMD,
    NAVIGATION_STATUS,
    NAVIGATION_UPDATE_COORDINATE,
    NAVIGATION_SET_TARGET
)

TOLERANCE_IN_METERS = 5.0


class NavigationActionClient(Node):

    def __init__(self):
        super().__init__("navigation_action_client")
        self.logger = self.get_logger()
        self._action_client = ActionClient(self, Navigate, "navigate")

        self.update_goals_subscriber = self.create_subscription(
            Coordinate, NAVIGATION_UPDATE_COORDINATE, self.update_goals, 10
        )

        self.set_target_subscriber = self.create_subscription(
            Coordinate, NAVIGATION_SET_TARGET, self.set_goal, 10
        )

        self.navigation_cmd_subscriber = self.create_subscription(
            Bool, NAVIGATION_CMD, self.handle_nav_cmd, 10
        )

        self.nav_publisher = self.create_publisher(NavigationStatus, NAVIGATION_STATUS, 10)

        self.goals = deque()
        self.goals_in_progress = deque()
        self.goals_completed = deque()
        self.auto_mode = False

        self.logger.info("Navigation client started")

    def update_goals(self, msg):
        goal_msg = Navigate.Goal()
        goal_msg.lat = msg.lat
        goal_msg.lon = msg.lon

        self.goals.append(goal_msg)
        self.logger.info(f"Updated goals, new goal lat: {goal_msg.lat}, lon: {goal_msg.lon}")
        self.logger.info(f"Currently {len(self.goals)} goals in queue.")
    
    def set_goal(self, msg):
        # Create goal and insert it from the left into the goal queue
        goal_msg = Navigate.Goal()
        goal_msg.lat = msg.lat
        goal_msg.lon = msg.lon
        
        self.logger.info(f"New target recieved with lat: {msg.lat} and lon {msg.lon}")
        self.goals.appendleft(goal_msg)

        # Now we need to cancel all goals sent to server and resend them to have the correct order
        self.cancel_goals_in_progress()
        self.send_goals_to_server()

    def handle_nav_cmd(self, msg):
        self.auto_mode = msg.data

        self.logger.info(f"Navigation mode command recieved, auto mode set to {self.auto_mode}")

        goals_already_in_progress = len(self.goals) == (len(self.goals_in_progress) + len(self.goals_completed))
        goals_in_queue = len(self.goals) > 0

        if self.auto_mode and goals_in_queue and not goals_already_in_progress:
            self.logger.info(f"Sending {len(self.goals)} goals to action server")
            self.send_goals_to_server()

        if not self.auto_mode and len(self.goals_in_progress) > 0:
            self.logger.info(f"Cancelling {len(self.goals_in_progress)} goals in progress")
            self.cancel_goals_in_progress()

    def send_goals_to_server(self):
        for goal_msg in self.goals:
            self.send_goal(goal_msg)

    def cancel_goals_in_progress(self):
        goal_handle = self.goals_in_progress.popleft()
        goal_handle.cancel_goal_async()
        self.goals_in_progress.clear()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.info("Goal server is unable to process goal")
            return

        self.logger.info("Goal was accepted by server")
        self.goals_in_progress.append(goal_handle)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        self.logger.info(f"Got a result callback from server {status}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            result = future.result().result

            self.logger.info(f"Goal finished successfully, now coordinates at lat: {result.lat} lon: {result.lon}")
            
            self.goals_completed.append(self.goals.popleft())
            self.goals_in_progress.popleft()
        elif status == GoalStatus.STATUS_CANCELED:
            self.logger.info(f"Goal number {len(self.goals_completed) + 1} has been canceled")


    def feedback_callback(self, feedback):
        nav_msg = NavigationStatus()
        nav_msg.auto_mode_enabled = self.auto_mode
        nav_msg.meters_to_target = feedback.feedback.distance_to_target
        nav_msg.tolerance_in_meters = TOLERANCE_IN_METERS
        coordinate = Coordinate()
        coordinate.lat = self.goals[0].lat
        coordinate.lon = self.goals[0].lon
        nav_msg.next_target = [coordinate]

        self.nav_publisher.publish(nav_msg)

    def send_goal(self, goal_msg):
        self.logger.info(f"Waiting for action server...")
        self._action_client.wait_for_server()

        self.logger.info(f"Sending goal request with lat: {goal_msg.lat} and lon: {goal_msg.lon}")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
    
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor=executor)


if __name__ == "__main__":
    main()
