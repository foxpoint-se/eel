#!/usr/bin/env python3
from time import time

import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ImuStatus, Coordinate, PressureStatus, HistoryEvent, Coordinate, HistoryEventList, ModemStatus
from ..utils.topics import (
    MODEM_STATUS,
    IMU_STATUS,
    LOCALIZATION_STATUS,
    PRESSURE_STATUS,
    HISTORY_EVENTS
)


class DataLogger(Node):
    def __init__(self):
        super().__init__("data_logger_node")
        self.logger = self.get_logger()

        self.depth = 0.0
        self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_depth_status_msg, 10
        )

        self.pitch = 0.0
        self.heading = 0.0
        self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_status_msg, 10
        )

        self.latitude = 0.0
        self.longitude = 0.0
        self.create_subscription(
            Coordinate, LOCALIZATION_STATUS, self.handle_gnss_status_msg, 10
        )

        self.modem_reg_status = 0.0
        self.modem_signal_strength = 0.0
        self.create_subscription(
            ModemStatus, MODEM_STATUS, self.handle_modem_status_msg, 10
        )

        self.history_event_publisher = self.create_publisher(
            HistoryEventList, HISTORY_EVENTS, 10
        )
    
        self.history_event_loggs = []
        self.updater = self.create_timer(2.0, self.update)

        self.logger.info("Data logger node started")

    def update(self):
        # First check if the depth would allow us to have a back end connection
        should_logg_data = False
        valid_connectivity_depth = self.depth < 0.2
        
        # If the depth could possibly allow us to have a backend connection, query mode for signal strength
        if valid_connectivity_depth:
            backend_connectivity = self.modem_reg_status == 1 and self.modem_signal_strength >= 16

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

    def logg_history_event(self):
        self.logger.debug(f"Adding data, now {len(self.history_event_loggs) + 1} entries")

        msg = HistoryEvent()
        msg.recorded_at = int(time())
        msg.depth = self.depth
        msg.pitch = self.pitch
        msg.heading = self.heading

        coordinate = Coordinate()
        coordinate.lat = self.latitude
        coordinate.lon = self.longitude
        msg.coordinate = coordinate

        self.history_event_loggs.append(msg)

    def handle_depth_status_msg(self, msg):
        self.depth = msg.depth

    def handle_imu_status_msg(self, msg):
        self.pitch = msg.pitch
        self.heading = msg.heading

    def handle_gnss_status_msg(self, msg):
        self.latitude = msg.lat
        self.longitude = msg.lon

    def handle_modem_status_msg(self, msg):
        self.modem_reg_status = msg.reg_status
        self.modem_signal_strength = msg.signal_strength

    def publish_history_events(self):
        self.logger.info(f"Sending {len(self.history_event_loggs)} events on topic {HISTORY_EVENTS}")

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
