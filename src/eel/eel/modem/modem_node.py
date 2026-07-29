#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from eel_interfaces.msg import ModemStatus

from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import MODEM_STATUS
from .modem_source import ModemSource


class ModemNode(Node):
    def __init__(self) -> None:
        super().__init__("modem_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.modem_publisher = self.create_publisher(ModemStatus, MODEM_STATUS, 10)
        self.logger = self.get_logger()

        sensor: ModemSource
        if not self.should_simulate:
            from .modem_sensor import ModemSensor

            sensor = ModemSensor()
        else:
            from .modem_simulator import ModemSimulator

            sensor = ModemSimulator(self)

        self.sensor = sensor

        reg_status = self.sensor.get_registration_status()
        signal_strength = self.sensor.get_received_signal_strength_indicator()
        if not reg_status or not signal_strength:
            raise Exception("Could not start modem node. Not getting registration status and/or signal strength.")

        self.update_modem_timer = self.create_timer(2.0, self.read_and_publish_modem_status)

        self.logger.info(f"{'Simulate ' if self.should_simulate else ''}Modem node started")

    def read_and_publish_modem_status(self) -> None:
        reg_status = self.sensor.get_registration_status()
        signal_strength = self.sensor.get_received_signal_strength_indicator()
        if reg_status is None or signal_strength is None:
            return

        check_connectivity = reg_status == 1 and signal_strength > 10
        if check_connectivity:
            connectivity = self.sensor.ping()
        else:
            connectivity = False

        msg = ModemStatus()
        msg.reg_status = reg_status
        msg.signal_strength = signal_strength
        msg.connectivity = connectivity

        self.modem_publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ModemNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
