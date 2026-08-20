"""Leakage logic: leakage/raw → leakage/status (pass-through)."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import LEAKAGE_RAW, LEAKAGE_STATUS


class LeakageNode(Node):
    def __init__(self) -> None:
        super().__init__("leakage")
        self.create_subscription(Bool, LEAKAGE_RAW, self._handle_raw, 10)
        self._pub = self.create_publisher(Bool, LEAKAGE_STATUS, 10)
        self.get_logger().info(f"Leakage started (listening on {LEAKAGE_RAW})")

    def _handle_raw(self, msg: Bool) -> None:
        out = Bool()
        out.data = bool(msg.data)
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LeakageNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
