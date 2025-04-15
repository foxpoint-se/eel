from collections import deque
from time import time
from typing import Deque, List, Sequence, cast

import rclpy
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.node import Node

from eel_interfaces.action import Navigate
from eel_interfaces.action._navigate import Navigate_FeedbackMessage
from eel_interfaces.msg import (
    NavigationStatus,
    NavigationMission,
    Coordinate,
    BatteryStatus,
    NavigationAssignment,
)
from ..utils.topics import (
    BATTERY_STATUS,
    LEAKAGE_STATUS,
    NAVIGATION_CMD,
    NAVIGATION_STATUS,
    NAVIGATION_LOAD_MISSION,
    GNSS_STATUS,
)
from ..utils.constants import NavigationMissionStatus
from .common import get_2d_distance_from_coords

GNSS_TIMEOUT_S = 300  # 5 minutes
MISSION_TIMEOUT_S = 3600  # 1 hour


def create_waypoint_goal(assignment: NavigationAssignment) -> Navigate.Goal:
    result = Navigate.Goal()
    result.next_coordinate = assignment.coordinate
    result.type = Navigate.Goal.TYPE_WAYPOINT
    result.optional_sync_time = []
    result.next_coordinate_depth = [assignment.target_depth]
    return result


def create_surface_goal(next_coordinate: Coordinate) -> Navigate.Goal:
    result = Navigate.Goal()
    result.next_coordinate = next_coordinate
    result.type = Navigate.Goal.TYPE_SURFACE_SYNC
    # TODO: I don't think this is used in server?
    result.optional_sync_time = [6.0]
    # TODO: bad name. since it is not used when surface
    result.next_coordinate_depth = []
    return result


MINIMUM_SYNC_DISTANCE_METERS = 15.0


def create_goals(assignments: Sequence[NavigationAssignment]) -> Deque[Navigate.Goal]:
    result: Deque[Navigate.Goal] = deque()
    for index, elem in enumerate(assignments):
        result.append(create_waypoint_goal(elem))
        has_next = index < (len(assignments) - 1)
        if elem.sync_after is True and has_next:
            next = assignments[index + 1]
            result.append(create_surface_goal(next.coordinate))

    return result


def is_waypoint_goal(goal: Navigate.Goal) -> bool:
    return goal.type == Navigate.Goal.TYPE_WAYPOINT


def extract_waypoints_from_goals(goals: Deque[Navigate.Goal]) -> List[Coordinate]:
    waypoints: List[Coordinate] = []
    for goal in goals:
        if is_waypoint_goal(goal):
            waypoints.append(goal.next_coordinate)
    return waypoints


def calculate_mission_distance_meters(goals: Deque[Navigate.Goal]) -> float:
    waypoints = extract_waypoints_from_goals(goals)
    total = 0.0
    for index, elem in enumerate(waypoints):
        has_next = index < (len(waypoints) - 1)
        if has_next:
            next = waypoints[index + 1]
            distance_to_next = get_2d_distance_from_coords(elem, next)
            total += distance_to_next
    return total


class NavigationActionClient(Node):

    def __init__(self):
        super().__init__("navigation_action_client", parameter_overrides=[])
        self.logger = self.get_logger()
        self._action_client = ActionClient(self, Navigate, "navigate")

        self.update_goals_subscriber = self.create_subscription(
            NavigationMission, NAVIGATION_LOAD_MISSION, self.set_mission, 10
        )

        self.navigation_cmd_subscriber = self.create_subscription(
            Bool, NAVIGATION_CMD, self.handle_nav_cmd, 10
        )

        self.battery_status_subscriber = self.create_subscription(
            BatteryStatus, BATTERY_STATUS, self.handle_battery_status, 10
        )

        self.leakage_status_subscriber = self.create_subscription(
            Bool, LEAKAGE_STATUS, self.handle_leakage_status, 10
        )

        self.aborter = self.create_timer(
            1.0 / 2, self.check_mission_abort_status
        )

        self.gnss_status_subscriber = self.create_subscription(
            Coordinate, GNSS_STATUS, self.handle_gnss_status, 10
        )

        self.nav_publisher = self.create_publisher(
            NavigationStatus, NAVIGATION_STATUS, 10
        )

        self.updater = self.create_timer(1.0, self.publish_status)

        self.last_seen_battery_level = 100.0
        self.last_seen_leakage = False


        self.goals: Deque[Navigate.Goal] = deque()
        self.goal_handles = []
        self.mission_status = NavigationMissionStatus.WAITING_FOR_MISSION
        self.mission_start_time = None
        self.goals_in_progress = False
        self.auto_mode = False
        self.meters_to_next_target: float = 0.0
        self.mission_total_meters: float = 0.0
        self.last_gnss_update_time: float = time()

        self.logger.info("Navigation client started")

    def publish_status(self) -> None:
        nav_msg = NavigationStatus()
        nav_msg.auto_mode_enabled = self.auto_mode
        nav_msg.meters_to_target = self.meters_to_next_target
        nav_msg.mission_total_meters = self.mission_total_meters
        nav_msg.waypoints_left = extract_waypoints_from_goals(self.goals)
        nav_msg.count_goals_left = len(self.goals)
        nav_msg.mission_status = self.mission_status.value
        self.nav_publisher.publish(nav_msg)

    def set_mission(self, msg: NavigationMission) -> None:
        """Takes a list of coordinates and converts them to a list of navigation goals."""
        coordinates = cast(List[NavigationAssignment], msg.assignments)

        if len(coordinates) == 0:
            self.goals.clear()
        else:
            self.goals = create_goals(coordinates)

        self.mission_status = NavigationMissionStatus.MISSION_AQUIRED
        self.mission_total_meters = calculate_mission_distance_meters(self.goals)
        self.logger.info(
            f"Set mission with {len(self.goals)} goals over {self.mission_total_meters} meters."
        )

    def handle_nav_cmd(self, msg: Bool) -> None:
        """Handler for nav cmd messages, either automode or manual as bool."""
        new_auto_mode = msg.data

        self.logger.info(f"Navigation mode command recieved:  {new_auto_mode}")

        if new_auto_mode == True and self.goals_in_progress:
            self.logger.info(
                "Goals already set and in progress, cancel auto mode and set new goals."
            )

        if (
            (new_auto_mode == True)
            and (len(self.goals) > 0)
            and not self.goals_in_progress
        ):
            self.logger.info(
                f"Mission started. Mission contains {len(self.goals)} goals."
            )
            self.start_mission()

        if (new_auto_mode == False) and self.goals_in_progress:
            self.logger.info(f"Cancelling goals in progress.")
            self.cancel_goals_in_progress()

        self.auto_mode = new_auto_mode

    def handle_battery_status(self, msg: BatteryStatus) -> None:
        """Updates the last seen battery level variable."""
        self.last_seen_battery_level = msg.voltage_percent

    def handle_leakage_status(self, msg: Bool) -> None:
        self.last_seen_leakage = msg.data

    def check_mission_abort_status(self) -> None:
        """Smelly function that does two things, first check incoming leakage status message,
        then checks battery level and mission time. These are all sources for mission aborts,
        if any of them is in a trigger state then cancel current mission."""
        battery_level_threshold = 0.10
        battery_level_low = self.last_seen_battery_level < battery_level_threshold

        if self.mission_start_time:
            mission_time_exceeded = (
                int(time() - self.mission_start_time) > MISSION_TIMEOUT_S
            )
        else:
            mission_time_exceeded = False

        leakage_detected = self.last_seen_leakage

        gnss_timeout = (time() - self.last_gnss_update_time) > GNSS_TIMEOUT_S

        # TODO: add leakage detected to this check
        if any([battery_level_low, mission_time_exceeded, gnss_timeout]) and self.goals_in_progress:
            self.logger.info(
                f"Battery low: {battery_level_low}\t"
                f"Mission time exceeded: {mission_time_exceeded}\t"
                f"GNSS timeout: {gnss_timeout}"
            )
            self.logger.info("Aborting mission")
            self.auto_mode = False
            self.cancel_goals_in_progress()

    def handle_gnss_status(self, msg: Coordinate) -> None:
        """Updates the current GNSS position."""
        self.last_gnss_update_time = time()

    def start_mission(self):
        """Method for starting the goal processing, used for starting the iteration."""
        self.send_goal(self.goals[0])
        self.goals_in_progress = True
        self.mission_start_time = time()
        self.mission_status = NavigationMissionStatus.MISSION_STARTED

    def cancel_goals_in_progress(self):
        """Method to cancel all the ongoing actions"""
        for handel in self.goal_handles:
            handel.cancel_goal_async()

        self.goal_handles = []
        self.goals_in_progress = False
        self.mission_start_time = None
        self.mission_status = NavigationMissionStatus.MISSION_CANCELLED

    def goal_response_callback(self, future):
        """Call back for when client has sent a goal to the server, will be accepted / rejected"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.logger.warning("Goal server is unable to process goal")
            return

        self.logger.debug("Goal was accepted by server")
        self.goal_handles.append(goal_handle)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Call back method for when the action server reports that is has finished a goal."""
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            result = future.result().result
            self.logger.info(f"Goal finished successfully")

            self.goals.popleft()

            if len(self.goals) == 0:
                self.logger.info(f"Finished mission, no more goals left.")
                self.goals_in_progress = False
                self.mission_start_time = None
                self.auto_mode = False
                self.mission_total_meters = 0.0
                self.mission_status = NavigationMissionStatus.MISSION_FINISHED
            else:
                self.send_goal(self.goals[0])

        elif status == GoalStatus.STATUS_CANCELED:
            self.logger.info(f"Goal was cancelled")
            self.mission_status = NavigationMissionStatus.MISSION_CANCELLED

    def feedback_callback(self, feedback: Navigate_FeedbackMessage):
        """Feed back message from action server sent once per second when action server gets a gps position."""
        self.meters_to_next_target = feedback.feedback.distance_to_target

    def send_goal(self, goal_msg):
        """Method for sending a goal to the action server, returns a goal handle from the server."""
        self.logger.debug(f"Waiting for action server...")
        self._action_client.wait_for_server()

        self.logger.info(f"Sending goal request")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
