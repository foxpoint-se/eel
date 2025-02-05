from abc import ABC, abstractmethod
from math import sin, radians
from typing import Callable, TypedDict
from time import time
from .common import (
    LatLon,
    get_2d_distance,
    get_relative_bearing,
    get_next_rudder_turn,
    TOLERANCE_IN_METERS,
)
from ..utils.nav import get_closest_turn_direction


def cap_value(value, floor, ceiling):
    if value > max([ceiling, floor]):
        return max([ceiling, floor])
    elif value < min([ceiling, floor]):
        return min([ceiling, floor])
    return value


class AssignmentProgress(TypedDict):
    distance_to_target: float


class Assignment(ABC):
    @abstractmethod
    def start(self) -> None:
        """Set the assignment going"""
        pass

    @abstractmethod
    def step(
        self,
        current_position: LatLon,
        current_heading: float,
        current_depth: float,
        current_time_seconds: float,
    ) -> AssignmentProgress:
        """Perform a step of the assignment and return feedback."""
        pass

    @abstractmethod
    def get_is_done(self) -> bool:
        """Check if the assignment is complete."""
        pass


def has_passed_waypoint(prev_wp, cur_wp, vehicle_pos):
    # Extract coordinates
    x_prev, y_prev = prev_wp
    x_cur, y_cur = cur_wp
    x_veh, y_veh = vehicle_pos

    # Direction vector and vector to vehicle
    d_x = x_cur - x_prev
    d_y = y_cur - y_prev
    v_x = x_veh - x_prev
    v_y = y_veh - y_prev

    # Dot product and magnitude of direction vector
    dot = v_x * d_x + v_y * d_y
    mag_d_squared = d_x**2 + d_y**2

    # Normalized projection
    t = dot / mag_d_squared

    # Check if waypoint is passed
    return t > 1


class WaypointAndDepth(Assignment):
    def __init__(
        self,
        target_pos: LatLon,
        start_pos: LatLon,
        depth: float,
        on_set_motor: Callable[[float], None],
        on_set_rudder: Callable[[float], None],
        on_set_depth: Callable[[float], None],
    ) -> None:
        self.target_depth = depth
        self.target_pos = target_pos
        self.is_done = False
        self.on_set_motor = on_set_motor
        self.on_set_rudder = on_set_rudder
        self.on_set_depth = on_set_depth
        self._initial_bearing_to_target = get_relative_bearing(start_pos, target_pos)
        self._prev_wp = (start_pos["lat"], start_pos["lon"])
        self._curr_wp = (target_pos["lat"], target_pos["lon"])

    def step(
        self,
        current_position: LatLon,
        current_heading: float,
        current_depth: float,
        current_time_seconds: float,
    ) -> AssignmentProgress:
        distance_to_target = get_2d_distance(current_position, self.target_pos)

        vehicle_pos = (current_position["lat"], current_position["lon"])
        has_passed = has_passed_waypoint(self._prev_wp, self._curr_wp, vehicle_pos)
        print("HAS PASSED WAYPOINT?", has_passed)

        if distance_to_target <= TOLERANCE_IN_METERS or has_passed:
            self.is_done = True
            return {"distance_to_target": distance_to_target}

        bearing_to_target = get_relative_bearing(current_position, self.target_pos)

        # offset_angle = abs(bearing_to_target - self._initial_bearing_to_target) % 360
        # offset_angle = 360 - offset_angle if offset_angle > 180 else offset_angle

        # closest_turn = get_closest_turn_direction(
        #     self._initial_bearing_to_target, bearing_to_target
        # )
        # offset = closest_turn * offset_angle

        offset = bearing_to_target - self._initial_bearing_to_target

        # print(f"Calculated offset is: {offset}")

        cross_track_error = sin(radians(offset)) * distance_to_target
        # next_rudder_turn = cap_value(cross_track_error * 0.05, -1.0, 1.0)

        # print(f"Cross track error: {cross_track_error}m")
        # print(f"Next rudder turn: {next_rudder_turn}")

        # next_rudder_turn = get_next_rudder_turn(current_heading, bearing_to_target)
        # self.on_set_rudder(next_rudder_turn)

        # desired heading = bearing of path - min(corrective turn angle, 90)
        # where
        # corrective turn angle = kct * ect
        # where
        # kct = ???? kp-värde??
        # ect = avstånd till optimal path

        bearing_of_path = self._initial_bearing_to_target
        ect = cross_track_error
        kct = -7.0  # NOTE: -20 works pretty good
        # kct = -5.0
        corrective_turn_angle = ect * kct

        # NOTE: when capping here, we might get a value like -250, since it's smaller than 90
        # this results in it turning full circle when we have a large error
        # instead we're capping between -90 and 90. this way we're avoiding turning the wrong way

        # NOTE: we have not solved what to do if we have passed the point.
        # the eel will return to desired path, but since it has passed the waypoint,
        # it will keep on going away from it. just following desired angle.
        # we need to fix that somehow. like: if it has passed it (if some angle is greater than some
        # value or something?) then we should just head for the waypoint instead.
        # i dunno.

        # desired_heading = (bearing_of_path - min(corrective_turn_angle, 90)) % 360
        desired_heading = (
            bearing_of_path - cap_value(corrective_turn_angle, -90, 90)
        ) % 360
        # desired_heading = bearing_of_path - min(corrective_turn_angle, 90)

        print("bearing_of_path", bearing_of_path)
        print("corrective_turn_angle", corrective_turn_angle)
        print("desired_heading", desired_heading)

        next_rudder_turn = get_next_rudder_turn(current_heading, desired_heading)

        self.on_set_rudder(next_rudder_turn)

        return {"distance_to_target": distance_to_target}

    def start(self) -> None:
        self.on_set_depth(self.target_depth)
        self.on_set_motor(1.0)

    def get_is_done(self) -> bool:
        return self.is_done


class SurfaceAssignment(Assignment):
    def __init__(
        self,
        target_pos: LatLon,
        start_pos: LatLon,
        depth: float,
        on_set_motor: Callable[[float], None],
        on_set_rudder: Callable[[float], None],
        on_set_depth: Callable[[float], None],
    ) -> None:
        self.target_depth = depth
        self.target_pos = target_pos
        self.is_done = False
        self.on_set_motor = on_set_motor
        self.on_set_rudder = on_set_rudder
        self.on_set_depth = on_set_depth
        self.last_update_at = time()
        self.seconds_at_surface = 0.0
        self._initial_bearing_to_target = get_relative_bearing(start_pos, target_pos)

    def start(self) -> None:
        self.on_set_depth(0.0)
        self.on_set_motor(1.0)
        self.last_update_at = time()

    def step(
        self,
        current_position: LatLon,
        current_heading: float,
        current_depth: float,
        current_time_seconds: float,
    ) -> AssignmentProgress:
        distance_to_target = get_2d_distance(current_position, self.target_pos)
        time_diff = current_time_seconds - self.last_update_at

        if -1.0 < current_depth < 0.2:
            self.seconds_at_surface += time_diff
        else:
            self.seconds_at_surface = 0.0

        if self.seconds_at_surface >= 6.0:
            self.is_done = True
            return {"distance_to_target": distance_to_target}

        bearing_to_target = get_relative_bearing(current_position, self.target_pos)

        next_rudder_turn = get_next_rudder_turn(current_heading, bearing_to_target)

        print(f"Bearing to target: {bearing_to_target}")
        print(f"Distance to target: {distance_to_target}")
        print(f"Inital bearing to target: {self._initial_bearing_to_target}")

        offset_angle = bearing_to_target - self._initial_bearing_to_target

        print(f"Angle offset: {offset_angle}")

        # drift_in_meters =

        self.on_set_rudder(next_rudder_turn)
        self.last_update_at = time()
        return {"distance_to_target": distance_to_target}

    def get_is_done(self) -> bool:
        return self.is_done
