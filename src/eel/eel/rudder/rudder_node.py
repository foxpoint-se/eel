#!/usr/bin/env python3
import math
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
from ..utils.constants import SIMULATE_PARAM
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import (
    IMU_STATUS,
    RUDDER_STATUS,
    RUDDER_X_CMD,
    RUDDER_X_OFFSET,
    RUDDER_X_SET_OFFSET,
    RUDDER_Y_CMD,
    RUDDER_Y_OFFSET,
    RUDDER_Y_SET_OFFSET,
)
from ..utils.utils import clamp
from .actuator.actuator import get_xy_rudder
from .actuator.types import Vector2d

RUDDER_CMD_TIMEOUT_PARAM = "cmd_timeout_s"
DEFAULT_RUDDER_CMD_TIMEOUT_S = 1.0
WATCHDOG_CHECK_PERIOD_S = 0.2


class Rudders(Enum):
    RUDDER_X = 1
    RUDDER_Y = 2


def rotate_vector(vector: Vector2d, rotation_degrees: float) -> Vector2d:
    rotation_radians = math.radians(rotation_degrees)
    x = vector["x"]
    y = vector["y"]
    rotated_x = math.cos(rotation_radians) * x - math.sin(rotation_radians) * y
    rotated_y = math.sin(rotation_radians) * x + math.cos(rotation_radians) * y
    return {"x": rotated_x, "y": rotated_y}


class Rudder(Node):
    def __init__(self) -> None:
        super().__init__("rudder_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter(RUDDER_CMD_TIMEOUT_PARAM, DEFAULT_RUDDER_CMD_TIMEOUT_S)
        self.should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)
        self.cmd_timeout_s = require_positive_cmd_timeout(float(self.get_parameter(RUDDER_CMD_TIMEOUT_PARAM).value))
        self.logger = self.get_logger()

        pigpiod_host_parameter = "pigpiod_host"
        self.declare_parameter(pigpiod_host_parameter, "localhost")
        self.pigpiod_host = str(self.get_parameter(pigpiod_host_parameter).value)

        self.x_subscription = self.create_subscription(Float32, RUDDER_X_CMD, self.handle_x_cmd, 10)
        self.x_offset_subscription = self.create_subscription(Float32, RUDDER_X_SET_OFFSET, self.handle_x_offset, 10)
        self.x_offset_publisher = self.create_publisher(Float32, RUDDER_X_OFFSET, 10)
        self.y_subscription = self.create_subscription(Float32, RUDDER_Y_CMD, self.handle_y_cmd, 10)
        self.y_offset_subscription = self.create_subscription(Float32, RUDDER_Y_SET_OFFSET, self.handle_y_offset, 10)
        self.y_offset_publisher = self.create_publisher(Float32, RUDDER_Y_OFFSET, 10)
        self.rudder_status_publisher = self.create_publisher(Vector3, RUDDER_STATUS, 10)

        self.imu_subscription = self.create_subscription(ImuStatus, IMU_STATUS, self._handle_imu_msg, 10)

        self.current_x_cmd: float = float()
        self.current_y_cmd: float = float()
        self._last_cmd_at: float | None = None

        self.current_roll = 0.0
        self.current_rudder_status: Vector2d = {"x": 0.0, "y": 0.0}

        self.xy_rudder = get_xy_rudder({"simulate": self.should_simulate, "pigpiod_host": self.pigpiod_host})

        self.create_timer(WATCHDOG_CHECK_PERIOD_S, self._watchdog_tick)

        self.logger.info(
            "{}Rudder node started (cmd_timeout_s={}).".format(
                "SIMULATE " if self.should_simulate else "",
                self.cmd_timeout_s,
            )
        )

        # Send the initial offset values for rudder x and y for front end to pick up
        self.publish_rudder_offset_value(Rudders.RUDDER_X)
        self.publish_rudder_offset_value(Rudders.RUDDER_Y)

    def shutdown(self) -> None:
        self.xy_rudder.shutdown()

    def _handle_imu_msg(self, msg: ImuStatus) -> None:
        self.current_roll = msg.roll
        self.merge_and_handle_commands()

    def handle_x_cmd(self, msg: Float32) -> None:
        bounded = bounded_unit_cmd(msg.data)
        if bounded is None:
            self.logger.warning(f"Rejecting invalid rudder x cmd {msg.data}")
            return
        self.current_x_cmd = bounded
        self._refresh_cmd_watchdog()
        self.merge_and_handle_commands()

    def publish_rudder_offset_value(self, rudder_type: Rudders) -> None:
        if rudder_type == Rudders.RUDDER_X:
            offset_value = self.xy_rudder.get_x_rudder_offset_value()
            publisher = self.x_offset_publisher
        elif rudder_type == Rudders.RUDDER_Y:
            offset_value = self.xy_rudder.get_y_rudder_offset_value()
            publisher = self.y_offset_publisher
        else:
            self.logger.warning(f"Invalid rudder type {rudder_type}, valid types are {Rudders}")
            return

        rudder_offset_msg = Float32()
        rudder_offset_msg.data = float(offset_value)
        publisher.publish(rudder_offset_msg)

    def handle_x_offset(self, msg: Float32) -> None:
        offset_value_to_low = msg.data < self.xy_rudder.get_x_rudder_cap_min_value()
        offset_value_to_high = msg.data > self.xy_rudder.get_x_rudder_cap_max_value()

        if any([offset_value_to_high, offset_value_to_low]):
            self.logger.warning(f"Requested x offset value {msg.data} is outside of boundries")
            return

        self.logger.info(f"Rudder x offset value set to {msg.data}")
        self.xy_rudder.set_x_rudder_offset_value(msg.data)

        self.publish_rudder_offset_value(Rudders.RUDDER_X)

    def publish_x_rudder_offset_value(self) -> None:
        pass

    def handle_y_cmd(self, msg: Float32) -> None:
        bounded = bounded_unit_cmd(msg.data)
        if bounded is None:
            self.logger.warning(f"Rejecting invalid rudder y cmd {msg.data}")
            return
        self.current_y_cmd = bounded
        self._refresh_cmd_watchdog()
        self.merge_and_handle_commands()

    def _refresh_cmd_watchdog(self) -> None:
        if self.current_x_cmd == 0 and self.current_y_cmd == 0:
            self._last_cmd_at = None
            return
        self._last_cmd_at = monotonic()

    def _watchdog_tick(self) -> None:
        if not command_is_stale(self._last_cmd_at, monotonic(), self.cmd_timeout_s):
            return
        self.logger.warning("rudder cmd stale; centering rudder")
        self.current_x_cmd = 0.0
        self.current_y_cmd = 0.0
        self._last_cmd_at = None
        self.merge_and_handle_commands()

    def handle_y_offset(self, msg: Float32) -> None:
        offset_value_to_low = msg.data < self.xy_rudder.get_y_rudder_cap_min_value()
        offset_value_to_high = msg.data > self.xy_rudder.get_y_rudder_cap_max_value()

        if any([offset_value_to_high, offset_value_to_low]):
            self.logger.warning(f"Requested y offset value {msg.data} is outside of boundries")
            return

        self.logger.info(f"Rudder y offset value set to {msg.data}")
        self.xy_rudder.set_y_rudder_offset_value(msg.data)

        self.publish_rudder_offset_value(Rudders.RUDDER_Y)

    def merge_and_handle_commands(self) -> None:
        direction: Vector2d = {
            "x": self.current_x_cmd,
            "y": self.current_y_cmd,
        }
        self.calc_and_send(rudder_direction=direction, roll_degrees=self.current_roll)

    def calc_and_send(self, rudder_direction: Vector2d, roll_degrees: float) -> None:
        rotation = -roll_degrees
        compensated = rotate_vector(vector=rudder_direction, rotation_degrees=rotation)
        clamped: Vector2d = {
            "x": float(clamp(compensated["x"], -1, 1)),
            "y": float(clamp(compensated["y"], -1, 1)),
        }

        self.rudder_status = clamped

        self.xy_rudder.set_rudder(clamped)

        status_vector_msg = Vector3()
        status_vector_msg.x = clamped["x"]
        status_vector_msg.y = clamped["y"]
        self.rudder_status_publisher.publish(status_vector_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Rudder()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
