#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ..utils.topics import LEAKAGE_STATUS
from ..utils.constants import SIMULATE_PARAM


class Leakage(Node):
    def __init__(self):
        super().__init__("leakage_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.update_frequency = 1
        self.publisher = self.create_publisher(Bool, LEAKAGE_STATUS, 10)

        if not self.should_simulate:
            from .leakage_sensor import LeakageSensor

            self.sensor = LeakageSensor()
        else:
            from .leakage_sim import LeakageSimulator

            self.sensor = LeakageSimulator()
        
        self.poller = self.create_timer(1.0 / self.update_frequency, self.read_and_publish_sensor_value)
        self.get_logger().info(
            f"{'Simulate' if self.should_simulate else ''} Leakage node started.")
    
    def read_and_publish_sensor_value(self):
        msg = Bool()
        msg.data = self.sensor.read_sensor()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Leakage()
    rclpy.spin(node)
    rclpy.shutdown()
