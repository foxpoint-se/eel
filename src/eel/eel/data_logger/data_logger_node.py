#!/usr/bin/env python3
from time import time
from typing import List, Union

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from eel_interfaces.msg import (
    ImuStatus,
    Coordinate,
    PressureStatus,
    HistoryEvent,
    Coordinate,
    HistoryEventList,
    ModemStatus,
)
from ..utils.topics import (
    MODEM_STATUS,
    IMU_STATUS,
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

        # self.depth = 0.0
        self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10
        )

        # self.pitch = 0.0
        # self.heading = 0.0
        # self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_status_msg, 10)

        # self.latitude = 0.0
        # self.longitude = 0.0
        self.create_subscription(
            Coordinate, LOCALIZATION_STATUS, self.handle_location_msg, 10
        )

        # self.modem_reg_status = 0.0
        # self.modem_signal_strength = 0.0
        self.current_modem_status: Union[ModemStatus, None] = None
        self.create_subscription(
            ModemStatus, MODEM_STATUS, self.on_connectivity_msg, 10
        )

        self.history_event_publisher = self.create_publisher(
            HistoryEventList, HISTORY_EVENTS, 10
        )

        self.history_event_loggs = []
        # self.updater = self.create_timer(2.0, self.update)

        self.worker = self.create_timer(0.5, self.do_stuff)
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
        # print("HAS CONNECTION", self.has_connection)
        if self.has_connection:
            self.history_event_publisher.publish(msg)

        else:
            self.stored.append(msg)

    def update(self):
        # First check if the depth would allow us to have a back end connection
        should_logg_data = False
        valid_connectivity_depth = self.current_depth and self.current_depth < 0.2

        # If the depth could possibly allow us to have a backend connection, query mode for signal strength
        if valid_connectivity_depth:
            backend_connectivity = self.modem_reg_status == 1 and self.modem_signal_strength >= 13

            # If there is backend connectivity and there is a active log file, then send it and delete log file
            if backend_connectivity and self.history_event_loggs:
                self.publish_history_events()
                self.history_event_loggs = []

            # If we are on a good depth yet there is no back end connectivity, then logg data
            if not backend_connectivity:
                should_logg_data = True

        if not valid_connectivity_depth:
            should_logg_data = True

        # If we should logg data, first check if there is a active logg file and then logg the data to file
        if should_logg_data:
            self.logg_history_event()

    def do_stuff(self) -> None:
        # self.recorder.update_3d_position()
        # if self.has_connection:
        #     self.recorder

        # self.recorder.step()
        if self.current_coord and self.current_depth:
            new_pos = TimedCoord3d(
                coord=Coord3d(
                    x=self.current_coord.lat,
                    y=self.current_coord.lon,
                    z=self.current_depth,
                ),
                created_at=time(),
            )
            # print("STEPPING", new_pos)
            self.recorder.step(new_pos=new_pos)
        # print("ALL", len(self.recorder.get_finalized_segments()))
        in_progress = self.recorder.get_segment_in_progress()
        length = 0 if in_progress is None else len(in_progress["polyline"])
        # print("IN PROGRESS", length)

    def handle_publish(self, segments: List[Segment]) -> None:
        pass

    def on_get_all_cmd(self, msg: Bool) -> None:
        pass

    def on_connectivity_msg(self, modem_status: ModemStatus) -> None:
        # print("GETTING MODEM STATUS", modem_status.connectivity)
        # if self.has_connection != modem_status.connectivity:
        self.has_connection = modem_status.connectivity
        if self.has_connection:
            for msg in self.stored:
                self.history_event_publisher.publish(msg)
                self.stored = []
                # self.recorder.flush()

    def logg_history_event(self):
        self.logger.debug(
            f"Adding data, now {len(self.history_event_loggs) + 1} entries"
        )

        msg = HistoryEvent()
        msg.recorded_at = int(time())
        msg.depth = self.current_depth
        msg.pitch = 0.0
        msg.heading = 0.0

        coordinate = Coordinate()
        # coordinate.lat = self.current_coord.lat
        # coordinate.lon = self.current_coord.lon
        msg.coordinate = coordinate

        self.history_event_loggs.append(msg)

    def handle_pressure_msg(self, msg: PressureStatus):
        self.current_depth = msg.depth

    # def handle_imu_status_msg(self, msg):
    #     self.pitch = msg.pitch
    #     self.heading = msg.heading

    def handle_location_msg(self, msg: Coordinate) -> None:
        self.current_coord = msg

    def handle_modem_status_msg(self, msg):
        self.modem_reg_status = msg.reg_status
        self.modem_signal_strength = msg.signal_strength

    def publish_history_events(self):
        self.logger.info(
            f"Sending {len(self.history_event_loggs)} events on topic {HISTORY_EVENTS}"
        )

        msg = HistoryEventList()
        msg.history_events = self.history_event_loggs

        self.history_event_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    data_logger_node = DataLogger()
    rclpy.spin(data_logger_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
