from abc import ABC, abstractmethod
from typing import Callable, TypedDict
from time import time
from .common import LatLon, get_2d_distance, get_relative_bearing, get_next_rudder_turn


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


TOLERANCE_IN_METERS = 5.0


class WaypointAndDepth(Assignment):
    def __init__(
        self,
        target_pos: LatLon,
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

    def step(
        self,
        current_position: LatLon,
        current_heading: float,
        current_depth: float,
        current_time_seconds: float,
    ) -> AssignmentProgress:
        distance_to_target = get_2d_distance(current_position, self.target_pos)

        if distance_to_target <= TOLERANCE_IN_METERS:
            self.is_done = True
            return {"distance_to_target": distance_to_target}

        bearing_to_target = get_relative_bearing(current_position, self.target_pos)

        next_rudder_turn = get_next_rudder_turn(current_heading, bearing_to_target)
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
        self.on_set_rudder(next_rudder_turn)
        self.last_update_at = time()
        return {"distance_to_target": distance_to_target}

    def get_is_done(self) -> bool:
        return self.is_done
