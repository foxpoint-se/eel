#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.srv import ModemStatus, ModemPing
from ..utils.constants import SIMULATE_PARAM

import requests
from requests.exceptions import ConnectionError, ConnectTimeout


class ModemNode(Node):
    def __init__(self):
        super().__init__("modem_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.status_srv = self.create_service(ModemStatus, "modem_status", self.get_modem_status)
        self.ping_srv = self.create_service(ModemPing, "modem_ping", self.ping)

        self.logger = self.get_logger()

        if not self.should_simulate:
            from .modem_sensor import ModemSensor

            self.sensor = ModemSensor()

        elif self.should_simulate:
            from .modem_simulator import ModemSimulator

            self.sensor = ModemSimulator()

        reg_status = self.sensor.get_registration_status()
        signal_strength = self.sensor.get_received_signal_strength_indicator()
        if not reg_status or not signal_strength:
            raise Exception(
                "Could not start modem node. Not getting registration status and/or signal strength."
            )

        self.logger.info(
            f"{'Simulate ' if self.should_simulate else ''}Modem node started"
        )

    def get_modem_status(self, request, response):
        response.reg_status = self.sensor.get_registration_status()
        response.signal_strength = self.sensor.get_received_signal_strength_indicator()

        self.logger.info(f"Reg status: {response.reg_status}, Signal strength {response.signal_strength}")                  

        return response
    
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
