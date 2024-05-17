#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ModemStatus
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import MODEM_STATUS

import requests
from requests.exceptions import ConnectionError, ConnectTimeout

class ModemNode(Node):
    def __init__(self):
        super().__init__("modem_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.modem_publisher = self.create_publisher(ModemStatus, MODEM_STATUS, 10)
        self.logger = self.get_logger()

        if not self.should_simulate:
            from .modem_sensor import ModemSensor

            self.sensor = ModemSensor()

        elif self.should_simulate:
            from .modem_simulator import ModemSimulator

            self.sensor = ModemSimulator(self)

        reg_status = self.sensor.get_registration_status()
        signal_strength = self.sensor.get_received_signal_strength_indicator()
        if not reg_status or not signal_strength:
            raise Exception(
                "Could not start modem node. Not getting registration status and/or signal strength."
            )

        self.update_modem_timer = self.create_timer(2.0, self.read_and_publish_modem_status)

        self.logger.info(
            f"{'Simulate ' if self.should_simulate else ''}Modem node started"
        )

    def read_and_publish_modem_status(self):
        msg = ModemStatus()
        msg.reg_status = self.sensor.get_registration_status()
        msg.signal_strength = self.sensor.get_received_signal_strength_indicator()

        self.modem_publisher.publish(msg)

    def ping(self, request, response):
        google_dns_server_url = "https://8.8.8.8"
        acceptable_response_time = 1.0

        try:
            req_response = requests.get(google_dns_server_url, timeout=acceptable_response_time)
            status_code = req_response.status_code
            response_time = req_response.elapsed.total_seconds()
        except (ConnectionError, ConnectTimeout) as e:
            status_code = 408
            response_time = -1.0

        self.logger.info(f"Pinged {google_dns_server_url}, status code: {status_code} with response time {response_time}")

        response.status_code = int(status_code)
        response.response_time = float(response_time)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModemNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
