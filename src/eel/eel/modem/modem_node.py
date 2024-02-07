#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ModemStatus
from .modem_sensor import ModemSensor
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import MODEM_STATUS


class ModemNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("modem_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.status_publisher = self.create_publisher(ModemStatus, MODEM_STATUS, 10)

        # hertz (publications per second)
        self.update_frequency = 0.10

        if not self.should_simulate:
            self.sensor = ModemSensor()
        
        if self.should_simulate:
            raise ValueError("Simulation of Modem not yet implemented")
        
        reg_status = self.sensor.get_registration_status()
        signal_strength = self.sensor.get_received_signal_strength_indicator()
        if not reg_status or not signal_strength:
            raise Exception("Could not start modem node. Not getting registration status and/or signal strength.")

        self.updater = self.create_timer(1.0 / self.update_frequency, self.publish_modem)

        self.get_logger().info(f"{'Simulate ' if self.should_simulate else ''}Modem node started")

    def publish_modem(self):
        reg_status = self.sensor.get_registration_status()
        signal_strength = self.sensor.get_received_signal_strength_indicator()

        msg = ModemStatus()
        msg.reg_status = reg_status
        msg.signal_strength = signal_strength

        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModemNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
