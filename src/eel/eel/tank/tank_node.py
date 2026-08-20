"""Tank logic: cmd + level → PID → pump_setpoint + TankStatus."""

from typing import Literal, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import TankStatus

from ..utils.actuator_bounds import bounded_tank_level
from ..utils.constants import CMD_TOPIC_PARAM, STATUS_TOPIC_PARAM
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.pid_controller import PidController
from ..utils.utils import clamp
from .tank_target import (
    DEFAULT_TANK_TARGET_CONFIG,
    RunningAverage,
    tank_stop_reason,
)
from .tank_topics import tank_level_topic, tank_pump_setpoint_topic

UPDATE_FREQUENCY = 10

TargetStatus = Literal["target_reached", "ceiling_reached", "floor_reached", "no_target", "adjusting"]


class TankNode(Node):
    def __init__(self) -> None:
        super().__init__("tank", parameter_overrides=[])
        self.declare_parameter(CMD_TOPIC_PARAM, "")
        self.declare_parameter(STATUS_TOPIC_PARAM, "")

        cmd_topic = str(self.get_parameter(CMD_TOPIC_PARAM).get_parameter_value().string_value)
        status_topic = str(self.get_parameter(STATUS_TOPIC_PARAM).get_parameter_value().string_value)
        if not cmd_topic or not status_topic:
            raise TypeError(f"Missing topic arguments ({CMD_TOPIC_PARAM}, {STATUS_TOPIC_PARAM})")

        level_topic = tank_level_topic(status_topic)
        pump_topic = tank_pump_setpoint_topic(status_topic)

        self.is_autocorrecting = False
        self.target_level: float | None = None
        self.target_status: TargetStatus = "no_target"
        self.current_level: float | None = None
        self.clamp_value = 0.0
        self.clamp_max_value = 1.0
        self.running_average = RunningAverage(10)
        self.tank_motor_pid = PidController(0.0, kP=8.0, kI=0.5, kD=2.5, output_min=-1.0, output_max=1.0)
        self._next_value_log_counter = 0

        self.create_subscription(Float32, cmd_topic, self._handle_tank_cmd, 10)
        self.create_subscription(Float32, level_topic, self._handle_level, 10)
        self._status_pub = self.create_publisher(TankStatus, status_topic, 10)
        self._pump_pub = self.create_publisher(Float32, pump_topic, 10)
        self.create_timer(1.0 / UPDATE_FREQUENCY, self._target_loop)

        self.get_logger().info(
            f"Tank started. CMD={cmd_topic} status={status_topic} level={level_topic} pump={pump_topic}"
        )

    def _publish_pump(self, value: float) -> None:
        msg = Float32()
        msg.data = value
        self._pump_pub.publish(msg)

    def _handle_level(self, msg: Float32) -> None:
        self.current_level = float(msg.data)

    def _handle_tank_cmd(self, msg: Float32) -> None:
        bounded_level = bounded_tank_level(msg.data)
        if bounded_level is None:
            self.get_logger().warning(f"Rejecting invalid tank target level {msg.data}")
            return
        target_level = clamp(
            bounded_level,
            DEFAULT_TANK_TARGET_CONFIG.level_floor,
            DEFAULT_TANK_TARGET_CONFIG.level_ceiling,
        )
        self.target_status = "adjusting"
        self.target_level = target_level
        self.get_logger().info(f"Setting set point {self.target_level}")
        self.tank_motor_pid.reset_cumulative_error()
        reset_ramp = abs(self.target_level - self.tank_motor_pid.set_point) > 0.05
        if reset_ramp:
            self.clamp_value = 0.0
        self.tank_motor_pid.update_set_point(self.target_level)

    def _publish_status(self, current_level: float | None) -> None:
        if current_level is None:
            return
        msg = TankStatus()
        msg.current_level = float(current_level)
        msg.target_level = []
        if self.target_level is not None:
            msg.target_level.append(self.target_level)
        msg.is_autocorrecting = self.is_autocorrecting
        msg.target_status = self.target_status
        self._status_pub.publish(msg)

    def _target_loop(self) -> None:
        if self.current_level is None:
            return

        self.running_average.add_sample(self.current_level)
        level_average = self.running_average.get_average()
        self._publish_status(self.current_level)

        if self.target_level is None:
            return

        stop_reason = tank_stop_reason(level_average, self.target_level)
        if stop_reason is not None:
            self._publish_pump(0.0)
            self.tank_motor_pid.reset_cumulative_error()
            self.target_status = stop_reason
            return

        if self.clamp_value < self.clamp_max_value:
            self.clamp_value = min(self.clamp_value + 0.05, self.clamp_max_value)
        self.tank_motor_pid.update_output_limits(-self.clamp_value, self.clamp_value)
        pid_value = self.tank_motor_pid.compute(level_average)

        self._next_value_log_counter += 1
        if self._next_value_log_counter >= 10:
            self.get_logger().info(f"next_value={pid_value} pid_value={pid_value}")
            self._next_value_log_counter = 0

        self._publish_pump(pid_value)

    def shutdown(self) -> None:
        self.target_level = None
        self.is_autocorrecting = False
        self._publish_pump(0.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TankNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
