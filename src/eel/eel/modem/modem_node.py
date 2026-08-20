"""Modem logic: ModemRaw → threshold + ping → ModemStatus.

Ping is injected at startup (HTTP on boat; no-op for CI).
"""

from collections.abc import Callable
from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import ModemRaw, ModemStatus

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import MODEM_RAW, MODEM_STATUS
from .modem_ping import http_ping


def noop_ping() -> bool:
    return False


class ModemNode(Node):
    def __init__(self, ping: Callable[[], bool]) -> None:
        super().__init__("modem")
        self._ping = ping
        self.create_subscription(ModemRaw, MODEM_RAW, self._handle_raw, 10)
        self._pub = self.create_publisher(ModemStatus, MODEM_STATUS, 10)
        self.get_logger().info(f"Modem started (listening on {MODEM_RAW})")

    def _handle_raw(self, msg: ModemRaw) -> None:
        reg_status = int(msg.reg_status)
        signal_strength = int(msg.signal_strength)
        check_connectivity = reg_status == 1 and signal_strength > 10
        connectivity = self._ping() if check_connectivity else False

        out = ModemStatus()
        out.reg_status = reg_status
        out.signal_strength = signal_strength
        out.connectivity = connectivity
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ModemNode(ping=http_ping)
    spin_node_until_shutdown(node)


def main_ci(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ModemNode(ping=noop_ping)
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
