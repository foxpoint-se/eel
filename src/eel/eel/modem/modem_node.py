#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.srv import ModemStatus
from ..utils.constants import SIMULATE_PARAM


class ModemNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("modem_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.srv = self.create_service(ModemStatus, "modem_status", self.get_modem_status)
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

        self.logger.info(f"Built response for incomin request for modem status.")
        self.logger.info(f"Reg status: {response.reg_status}, Signal strength {response.signal_strength}")                  

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModemNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
