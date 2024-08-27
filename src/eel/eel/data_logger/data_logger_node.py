#!/usr/bin/env python3
from time import time
from typing import List, Union

import rclpy
from rclpy.node import Node
from eel_interfaces.msg import (
    Coordinate,
    PressureStatus,
    HistoryEvent,
    Coordinate,
    HistoryEventList,
    ModemStatus,
)
from ..utils.topics import (
    MODEM_STATUS,
    LOCALIZATION_STATUS,
    PRESSURE_STATUS,
    HISTORY_EVENTS,
)
from .data_recorder import PathRecorder, Segment
from .common import Segment, TimedCoord3d, Coord3d


def to_history_event(coord: TimedCoord3d) -> HistoryEvent:
    ev = HistoryEvent()
    c = Coordinate()
    c.lat = coord["coord"]["x"]
    c.lon = coord["coord"]["y"]
    ev.coordinate = c
    ev.depth = coord["coord"]["z"]
    ev.heading = 0.0
    ev.pitch = 0.0
    # TODO: what format for recorded_at
    ev.recorded_at = int(coord["created_at"])
    return ev


def to_history_event_list(segment: Segment) -> HistoryEventList:
    event_list = HistoryEventList()
    event_list.history_events = [to_history_event(c) for c in segment["polyline"]]
    return event_list


class DataLogger(Node):
    def __init__(self):
        super().__init__("data_logger_node", parameter_overrides=[])
        self.logger = self.get_logger()
        self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10
        )
        self.create_subscription(
            Coordinate, LOCALIZATION_STATUS, self.handle_location_msg, 10
        )
        self.current_modem_status: Union[ModemStatus, None] = None
        self.create_subscription(
            ModemStatus, MODEM_STATUS, self.on_connectivity_msg, 10
        )
        self.history_event_publisher = self.create_publisher(
            HistoryEventList, HISTORY_EVENTS, 10
        )
        self.worker = self.create_timer(0.5, self.update_recorder)

        # TODO: we need to change seconds threshold, either here or when connectivity is back
        self.recorder = PathRecorder(
            meters_threshold=1.0,
            seconds_threshold=10.0,
            on_new_segment=self.handle_new_segment,
        )
        self.has_connection: bool = False
        self.current_coord: Union[Coordinate, None] = None
        self.current_depth: Union[float, None] = None
        self.stored: List[HistoryEventList] = []

        self.logger.info("Data logger node started")

    def handle_new_segment(self, segment: Segment) -> None:
        msg = to_history_event_list(segment)
        if self.has_connection:
            self.history_event_publisher.publish(msg)
        else:
            self.stored.append(msg)

    def update_recorder(self) -> None:
        if self.current_coord and self.current_depth:
            new_pos = TimedCoord3d(
                coord=Coord3d(
                    x=self.current_coord.lat,
                    y=self.current_coord.lon,
                    z=self.current_depth,
                ),
                created_at=time(),
            )
            self.recorder.step(new_pos=new_pos)

    def on_connectivity_msg(self, modem_status: ModemStatus) -> None:
        self.has_connection = modem_status.connectivity
        if self.has_connection:
            for msg in self.stored:
                self.history_event_publisher.publish(msg)
                self.stored = []

    def handle_pressure_msg(self, msg: PressureStatus) -> None:
        self.current_depth = msg.depth

    def handle_location_msg(self, msg: Coordinate) -> None:
        self.current_coord = msg


def main(args=None):
    rclpy.init(args=args)
    data_logger_node = DataLogger()
    rclpy.spin(data_logger_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
