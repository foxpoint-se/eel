from collections import deque
from time import time
from typing import Deque, List, Sequence, Union, cast

import rclpy
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
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
    PressureStatus,
)
from ..utils.topics import (
    BATTERY_STATUS,
    LEAKAGE_STATUS,
    NAVIGATION_CMD,
    NAVIGATION_STATUS,
    NAVIGATION_LOAD_MISSION,
    NAVIGATION_LOAD_NAMED_MISSION,
    GNSS_STATUS,
    PRESSURE_STATUS,
    LOCALIZATION_STATUS,
)
from ..utils.constants import NavigationMissionStatus
from .common import get_2d_distance_from_coords

GNSS_TIMEOUT_S = 300  # 5 minutes
MISSION_TIMEOUT_S = 3600  # 1 hour
MAX_DEPTH_METERS = 3.0
COUNT_MAX_DEPTHS = 3

def create_rotholmen_runt_2025_mission() -> NavigationMission:
    mission = NavigationMission()
    assignments: List[NavigationAssignment] = [
        NavigationAssignment(coordinate=Coordinate(lat=59.309024798716145, lon=17.97766066452895), target_depth=0.0, sync_after=False),
        NavigationAssignment(coordinate=Coordinate(lat=59.30938074193075, lon=17.976039505952492), target_depth=0.0, sync_after=False),
        NavigationAssignment(coordinate=Coordinate(lat=59.31003238216224, lon=17.974268041282897), target_depth=0.0, sync_after=False),
        NavigationAssignment(coordinate=Coordinate(lat=59.31124253803556, lon=17.973237370929635), target_depth=1.8, sync_after=True),
        NavigationAssignment(coordinate=Coordinate(lat=59.31260596522936, lon=17.972947494892782), target_depth=1.8, sync_after=True),
        NavigationAssignment(coordinate=Coordinate(lat=59.313246592863365, lon=17.97532018393514), target_depth=1.8, sync_after=True),
        NavigationAssignment(coordinate=Coordinate(lat=59.31326301905425, lon=17.977467413837743), target_depth=1.8, sync_after=True),
        NavigationAssignment(coordinate=Coordinate(lat=59.312304811316324, lon=17.979410656899567), target_depth=1.8, sync_after=True),
        NavigationAssignment(coordinate=Coordinate(lat=59.31118778078151, lon=17.977972012864825), target_depth=1.8, sync_after=True),
        NavigationAssignment(coordinate=Coordinate(lat=59.31004240514067, lon=17.97663803259078), target_depth=1.8, sync_after=False),
        NavigationAssignment(coordinate=Coordinate(lat=59.30951945130858, lon=17.975909267643953), target_depth=0.0, sync_after=False),
        NavigationAssignment(coordinate=Coordinate(lat=59.309320505648635, lon=17.979238878507378), target_depth=0.0, sync_after=False),
        NavigationAssignment(coordinate=Coordinate(lat=59.308889, lon=17.979588), target_depth=0.0, sync_after=False),
    ]    
    mission.assignments = assignments
    return mission


def create_waypoint_goal(assignment: NavigationAssignment, start: Coordinate) -> Navigate.Goal:
    result = Navigate.Goal()
    result.start = start
    result.goal = assignment.coordinate
    result.type = Navigate.Goal.TYPE_WAYPOINT
    result.optional_sync_time = []
    result.next_coordinate_depth = [assignment.target_depth]

    next_depth = assignment.target_depth
    # NOTE: if setting depth 0.0 in gui, we will set it to -0.3, for a bit more juice.
    if -0.01 < assignment.target_depth < 0.01:
        next_depth = -0.4

    result.next_coordinate_depth = [next_depth]
    return result


def create_surface_goal(next_coordinate: Coordinate, start: Coordinate) -> Navigate.Goal:
    result = Navigate.Goal()
    result.start = start
    result.goal = next_coordinate
    result.type = Navigate.Goal.TYPE_SURFACE_SYNC
    # TODO: I don't think this is used in server?
    result.optional_sync_time = [6.0]
    # TODO: bad name. since it is not used when surface
    result.next_coordinate_depth = []
    return result


MINIMUM_SYNC_DISTANCE_METERS = 15.0


def create_goals(assignments: Sequence[NavigationAssignment], current_position: Coordinate) -> Deque[Navigate.Goal]:
    result: Deque[Navigate.Goal] = deque()
    for index, elem in enumerate(assignments):

        has_prev = index > 0
        if has_prev:
            prev = assignments[index - 1]
            start = prev.coordinate
        else:
            start = current_position

        result.append(create_waypoint_goal(elem, start))
        has_next = index < (len(assignments) - 1)
        if elem.sync_after is True and has_next:
            next = assignments[index + 1]
            result.append(create_surface_goal(next.coordinate, elem.coordinate))

    print("goals", result)

    return result


def is_waypoint_goal(goal: Navigate.Goal) -> bool:
    return goal.type == Navigate.Goal.TYPE_WAYPOINT


def extract_waypoints_from_goals(goals: Deque[Navigate.Goal]) -> List[Coordinate]:
    waypoints: List[Coordinate] = []
    for goal in goals:
        if is_waypoint_goal(goal):
            waypoints.append(goal.goal)
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
        
        self.named_mission_subscriber = self.create_subscription(
            String, NAVIGATION_LOAD_NAMED_MISSION, self.load_named_mission, 10
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

        self.pressure_status_subscriber = self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_status, 10
        )

        self.aborter = self.create_timer(
            1.0 / 2, self.check_mission_abort_status
        )

        self.gnss_status_subscriber = self.create_subscription(
            Coordinate, GNSS_STATUS, self.handle_gnss_status, 10
        )

        self.localization_subscriber = self.create_subscription(
            Coordinate, LOCALIZATION_STATUS, self.handle_localization_update, 10
        )

        self.nav_publisher = self.create_publisher(
            NavigationStatus, NAVIGATION_STATUS, 10
        )

        self.updater = self.create_timer(1.0, self.publish_status)

        self.last_seen_battery_level = 100.0
        self.last_seen_leakage = False

        self.current_position: Union[Coordinate, None] = None


        self.goals: Deque[Navigate.Goal] = deque()
        self.goal_handles = []
        self.mission_status = NavigationMissionStatus.WAITING_FOR_MISSION
        self.mission_start_time = None
        self.goals_in_progress = False
        self.auto_mode = False
        self.meters_to_next_target: float = 0.0
        self.mission_total_meters: float = 0.0
        self.last_gnss_update_time: float = time()
        self.is_too_deep = False
        self.depth_history = deque(maxlen=COUNT_MAX_DEPTHS)

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
            if self.current_position is None:
                self.logger.info("Cannot create goals without current position.")
                return
            
            self.goals = create_goals(coordinates, self.current_position)

        self.mission_status = NavigationMissionStatus.MISSION_AQUIRED
        self.mission_total_meters = calculate_mission_distance_meters(self.goals)
        self.logger.info(
            f"Set mission with {len(self.goals)} goals over {self.mission_total_meters} meters."
        )

    def load_named_mission(self, msg: String) -> None:
        if msg.data == "rotholmen_runt_2025":
            mission = create_rotholmen_runt_2025_mission()
            self.set_mission(mission)
        else:
            self.logger.info(f"Unknown mission name {msg.data}")

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

    def handle_localization_update(self, msg: Coordinate) -> None:
        self.current_position = msg

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
        if any([battery_level_low, mission_time_exceeded, gnss_timeout, self.is_too_deep]) and self.goals_in_progress:
            self.logger.info(
                f"Battery low: {battery_level_low}\t"
                f"Mission time exceeded: {mission_time_exceeded}\t"
                f"GNSS timeout: {gnss_timeout}\t"
                f"Is too deep: {self.is_too_deep} {self.depth_history}"
            )
            self.logger.info("Aborting mission")
            self.auto_mode = False
            self.cancel_goals_in_progress()

    def handle_gnss_status(self, msg: Coordinate) -> None:
        """Updates the current GNSS position."""
        self.last_gnss_update_time = time()

    def handle_pressure_status(self, msg: PressureStatus) -> None:
        self.depth_history.append(msg.depth)
        over_limit_count = sum(d > MAX_DEPTH_METERS for d in self.depth_history)

        self.is_too_deep = (over_limit_count == COUNT_MAX_DEPTHS)
        

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
