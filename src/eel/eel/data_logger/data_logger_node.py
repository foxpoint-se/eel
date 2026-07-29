#!/usr/bin/env python3
from datetime import datetime, timezone
from time import time
from typing import List, Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import (
    Coordinate,
    ModemStatus,
    PressureStatus,
    SubmergedCoordinate,
    TracedRoute,
)

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import (
    LOCALIZATION_STATUS,
    MODEM_STATUS,
    PRESSURE_STATUS,
    ROUTE_TRACING_UPDATES,
)
from .common import Coord3d, Segment, TimedCoord3d
from .data_recorder import PathRecorder


def to_submerged_coordinate(coord: TimedCoord3d) -> SubmergedCoordinate:
    ev = SubmergedCoordinate()
    c = Coordinate()
    c.lat = coord["coord"]["lat"]
    c.lon = coord["coord"]["lon"]
    ev.coordinate = c
    ev.depth = coord["coord"]["depth"]
    return ev


def to_utc_string(timestamp: float) -> str:
    return datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()


def to_traced_route(segment: Segment) -> TracedRoute:
    route = TracedRoute()
    route.path = [to_submerged_coordinate(c) for c in segment["polyline"]]
    route.duration_seconds = segment["ended_at_seconds"] - segment["started_at_seconds"]
    route.xy_distance_covered_meters = segment["accumulated_distance"]
    route.average_depth_meters = sum([d.depth for d in route.path]) / len(route.path)
    route.started_at = to_utc_string(segment["started_at_seconds"])
    route.ended_at = to_utc_string(segment["ended_at_seconds"])
    return route


class DataLogger(Node):
    def __init__(self) -> None:
        super().__init__("data_logger_node", parameter_overrides=[])
        self.logger = self.get_logger()
        self.create_subscription(PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10)
        self.create_subscription(Coordinate, LOCALIZATION_STATUS, self.handle_location_msg, 10)
        self.current_modem_status: ModemStatus | None = None
        self.create_subscription(ModemStatus, MODEM_STATUS, self.on_connectivity_msg, 10)
        self.history_event_publisher = self.create_publisher(TracedRoute, ROUTE_TRACING_UPDATES, 10)
        self.worker = self.create_timer(1.0, self.update_recorder)

        self.recorder = PathRecorder(
            meters_threshold=1.0,
            seconds_threshold=10.0,
            on_new_segment=self.handle_new_segment,
        )
        self.has_connection: bool = False
        self.current_coord: Coordinate | None = None
        self.current_depth: float | None = None
        self.stored: List[TracedRoute] = []

        self.logger.info("Data logger node started")

    def handle_new_segment(self, segment: Segment) -> None:
        msg = to_traced_route(segment)
        if self.has_connection:
            self.history_event_publisher.publish(msg)
        else:
            self.stored.append(msg)

    def update_recorder(self) -> None:
        if self.current_coord and self.current_depth is not None:
            new_pos = TimedCoord3d(
                coord=Coord3d(
                    lat=self.current_coord.lat,
                    lon=self.current_coord.lon,
                    depth=self.current_depth,
                ),
                created_at=time(),
            )
            self.recorder.step(new_pos=new_pos)

    def on_connectivity_msg(self, modem_status: ModemStatus) -> None:
        self.has_connection = modem_status.connectivity
        if self.has_connection:
            self.recorder.set_thresholds(meters_threshold=1, seconds_threshold=3.0)
            for msg in self.stored:
                self.history_event_publisher.publish(msg)
                self.stored = []
        else:
            self.recorder.set_thresholds(meters_threshold=1, seconds_threshold=10.0)

    def handle_pressure_msg(self, msg: PressureStatus) -> None:
        self.current_depth = msg.depth

    def handle_location_msg(self, msg: Coordinate) -> None:
        self.current_coord = msg


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    data_logger_node = DataLogger()
    spin_node_until_shutdown(data_logger_node)


if __name__ == "__main__":
    main()
