"""Rudder logic: cmds/offsets/roll → status + rudder/setpoint.

Offsets live here; setpoint already includes offset for pure servo I/O.
"""

from enum import Enum
from time import monotonic
from typing import Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import ImuStatus

from ..motor.motor_watchdog import command_is_stale, require_positive_cmd_timeout
from ..utils.actuator_bounds import bounded_unit_cmd
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import (
    IMU_STATUS,
    RUDDER_SETPOINT,
    RUDDER_STATUS,
    RUDDER_X_CMD,
    RUDDER_X_OFFSET,
    RUDDER_X_SET_OFFSET,
    RUDDER_Y_CMD,
    RUDDER_Y_OFFSET,
    RUDDER_Y_SET_OFFSET,
)
from .actuator.actuator import default_x_options, default_y_options
from .actuator.types import Vector2d
from .rudder_math import roll_compensated_deflection, servo_command_with_offset

RUDDER_CMD_TIMEOUT_PARAM = "cmd_timeout_s"
DEFAULT_RUDDER_CMD_TIMEOUT_S = 1.0
WATCHDOG_CHECK_PERIOD_S = 0.2


class Rudders(Enum):
    RUDDER_X = 1
    RUDDER_Y = 2


class RudderNode(Node):
    def __init__(self) -> None:
        super().__init__("rudder")
        self.declare_parameter(RUDDER_CMD_TIMEOUT_PARAM, DEFAULT_RUDDER_CMD_TIMEOUT_S)
        self.cmd_timeout_s = require_positive_cmd_timeout(float(self.get_parameter(RUDDER_CMD_TIMEOUT_PARAM).value))
        self.logger = self.get_logger()

        self._x_offset = float(default_x_options["offset"])
        self._y_offset = float(default_y_options["offset"])
        self._x_cap_min = float(default_x_options["cap_min"])
        self._x_cap_max = float(default_x_options["cap_max"])
        self._y_cap_min = float(default_y_options["cap_min"])
        self._y_cap_max = float(default_y_options["cap_max"])

        self.create_subscription(Float32, RUDDER_X_CMD, self._handle_x_cmd, 10)
        self.create_subscription(Float32, RUDDER_Y_CMD, self._handle_y_cmd, 10)
        self.create_subscription(Float32, RUDDER_X_SET_OFFSET, self._handle_x_offset, 10)
        self.create_subscription(Float32, RUDDER_Y_SET_OFFSET, self._handle_y_offset, 10)
        self.create_subscription(ImuStatus, IMU_STATUS, self._handle_imu, 10)

        self._x_offset_pub = self.create_publisher(Float32, RUDDER_X_OFFSET, 10)
        self._y_offset_pub = self.create_publisher(Float32, RUDDER_Y_OFFSET, 10)
        self._status_pub = self.create_publisher(Vector3, RUDDER_STATUS, 10)
        self._setpoint_pub = self.create_publisher(Vector3, RUDDER_SETPOINT, 10)

        self._x_cmd = 0.0
        self._y_cmd = 0.0
        self._last_cmd_at: float | None = None
        self._roll_deg = 0.0

        self.create_timer(WATCHDOG_CHECK_PERIOD_S, self._watchdog_tick)
        self.logger.info(f"Rudder started (cmd_timeout_s={self.cmd_timeout_s})")
        self._publish_offset(Rudders.RUDDER_X)
        self._publish_offset(Rudders.RUDDER_Y)

    def _handle_imu(self, msg: ImuStatus) -> None:
        self._roll_deg = float(msg.roll)
        self._merge_and_publish()

    def _handle_x_cmd(self, msg: Float32) -> None:
        bounded = bounded_unit_cmd(msg.data)
        if bounded is None:
            self.logger.warning(f"Rejecting invalid rudder x cmd {msg.data}")
            return
        self._x_cmd = bounded
        self._refresh_cmd_watchdog()
        self._merge_and_publish()

    def _handle_y_cmd(self, msg: Float32) -> None:
        bounded = bounded_unit_cmd(msg.data)
        if bounded is None:
            self.logger.warning(f"Rejecting invalid rudder y cmd {msg.data}")
            return
        self._y_cmd = bounded
        self._refresh_cmd_watchdog()
        self._merge_and_publish()

    def _handle_x_offset(self, msg: Float32) -> None:
        if msg.data < self._x_cap_min or msg.data > self._x_cap_max:
            self.logger.warning(f"Requested x offset value {msg.data} is outside of boundries")
            return
        self.logger.info(f"Rudder x offset value set to {msg.data}")
        self._x_offset = float(msg.data)
        self._publish_offset(Rudders.RUDDER_X)
        self._merge_and_publish()

    def _handle_y_offset(self, msg: Float32) -> None:
        if msg.data < self._y_cap_min or msg.data > self._y_cap_max:
            self.logger.warning(f"Requested y offset value {msg.data} is outside of boundries")
            return
        self.logger.info(f"Rudder y offset value set to {msg.data}")
        self._y_offset = float(msg.data)
        self._publish_offset(Rudders.RUDDER_Y)
        self._merge_and_publish()

    def _publish_offset(self, rudder_type: Rudders) -> None:
        msg = Float32()
        if rudder_type == Rudders.RUDDER_X:
            msg.data = self._x_offset
            self._x_offset_pub.publish(msg)
        else:
            msg.data = self._y_offset
            self._y_offset_pub.publish(msg)

    def _refresh_cmd_watchdog(self) -> None:
        if self._x_cmd == 0 and self._y_cmd == 0:
            self._last_cmd_at = None
            return
        self._last_cmd_at = monotonic()

    def _watchdog_tick(self) -> None:
        if not command_is_stale(self._last_cmd_at, monotonic(), self.cmd_timeout_s):
            return
        self.logger.warning("rudder cmd stale; centering rudder")
        self._x_cmd = 0.0
        self._y_cmd = 0.0
        self._last_cmd_at = None
        self._merge_and_publish()

    def _merge_and_publish(self) -> None:
        direction: Vector2d = {"x": self._x_cmd, "y": self._y_cmd}
        deflection = roll_compensated_deflection(direction, self._roll_deg)

        status = Vector3()
        status.x = deflection["x"]
        status.y = deflection["y"]
        self._status_pub.publish(status)

        setpoint = Vector3()
        setpoint.x = servo_command_with_offset(deflection["x"], self._x_offset, self._x_cap_min, self._x_cap_max)
        setpoint.y = servo_command_with_offset(deflection["y"], self._y_offset, self._y_cap_min, self._y_cap_max)
        self._setpoint_pub.publish(setpoint)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RudderNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
