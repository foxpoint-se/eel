#!/usr/bin/env python3
import math
from typing import Literal, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import TankStatus

from ..utils.actuator_bounds import bounded_tank_level
from ..utils.constants import (
    CMD_TOPIC_PARAM,
    DIRECTION_PIN_PARAM,
    DISTANCE_SENSOR_CHANNEL_PARAM,
    MOTOR_PIN_PARAM,
    SIMULATE_PARAM,
    STATUS_TOPIC_PARAM,
    TANK_CEILING_VALUE_PARAM,
    TANK_FLOOR_VALUE_PARAM,
)
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.pid_controller import PidController
from ..utils.utils import clamp
from .tank_target import (
    DEFAULT_TANK_TARGET_CONFIG,
    RunningAverage,
    tank_stop_reason,
)
from .tank_utils.create_tank import create_tank

TANK_FILL_TIME_S = 22

# TANK_RANGE_MM = TANK_CEILING_MM - TANK_FLOOR_MM
# TANK_FILL_VELOCITY_MMPS = TANK_RANGE_MM / TANK_FILL_TIME_S

# Minimum update frequency should be higher than tank fill velocity divided by target tolerance.
# Otherwise we might miss the target, since the pump has rushed past the target area before checking again.
# Example:
# tank floor: 30 mm
# tank ceiling: 65 mm
# tank fill time: 22 s
# target tolerance: 0.03 (3%)
# tank range: ceiling - floor = 65 - 30 = 35 mm
# fill velocity: range / fill time = 35 / 22 = 1.6 mm/s
# tolerance in mm: tolerance * range = 0.03 * 35 = 1.05 mm
# This means that 1 update per second is not enough, since it will run 1.6 mm in a second, and therefore
# miss the tolerance span of 1.05 mm.
# Minimum update frequency: velocity / tolerance = 1.6 / 1.05 = 1.5 hz
# UPDATE_FREQUENCY = 10 should therefore be plenty.
UPDATE_FREQUENCY = 10

TANK_CALIBRATION_UNSET = float("nan")

TargetStatus = Literal["target_reached", "ceiling_reached", "floor_reached", "no_target", "adjusting"]


# NOTE: example usage
# OLD
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_rear/cmd \
#   -p status_topic:=/tank_rear/status -p motor_pin:=24 -p direction_pin:=25 \
#   -p tank_floor_mm:=15 -p tank_ceiling_mm:=63 -p xshut_pin_param:=21
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_front/cmd \
#   -p status_topic:=/tank_front/status -p motor_pin:=23 -p direction_pin:=18 \
#   -p tank_floor_mm:=15 -p tank_ceiling_mm:=63 -p xshut_pin_param:=21
# NEW
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_front/cmd \
#   -p status_topic:=/tank_front/status -p motor_pin:=23 -p direction_pin:=18 \
#   -p tank_floor_value:=4736 -p tank_ceiling_value:=18256 -p distance_sensor_channel:=1


class TankNode(Node):
    def __init__(self) -> None:
        super().__init__("tank_node", parameter_overrides=[])
        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter(CMD_TOPIC_PARAM, "")
        self.declare_parameter(STATUS_TOPIC_PARAM, "")
        self.declare_parameter(MOTOR_PIN_PARAM, -1)
        self.declare_parameter(DIRECTION_PIN_PARAM, -1)
        self.declare_parameter(DISTANCE_SENSOR_CHANNEL_PARAM, -1)
        self.declare_parameter(TANK_FLOOR_VALUE_PARAM, TANK_CALIBRATION_UNSET)
        self.declare_parameter(TANK_CEILING_VALUE_PARAM, TANK_CALIBRATION_UNSET)

        should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)
        motor_pin = int(self.get_parameter(MOTOR_PIN_PARAM).get_parameter_value().integer_value)
        direction_pin = int(self.get_parameter(DIRECTION_PIN_PARAM).get_parameter_value().integer_value)
        distance_sensor_channel = int(
            self.get_parameter(DISTANCE_SENSOR_CHANNEL_PARAM).get_parameter_value().integer_value
        )
        floor_value = float(self.get_parameter(TANK_FLOOR_VALUE_PARAM).get_parameter_value().double_value)
        ceiling_value = float(self.get_parameter(TANK_CEILING_VALUE_PARAM).get_parameter_value().double_value)

        if math.isnan(floor_value) or math.isnan(ceiling_value):
            raise TypeError(f"Missing calibration parameters ({TANK_FLOOR_VALUE_PARAM}, {TANK_CEILING_VALUE_PARAM})")

        self.is_autocorrecting: bool = False
        self.target_level: Optional[float] = None

        # only used for passing information to frontend
        self.target_status: TargetStatus = "no_target"

        self.current_level: Optional[float] = None

        self.clamp_value = 0.0
        self.clamp_max_value = 1.0

        self.running_average = RunningAverage(10)

        # float Kp = 0.2;
        # float Ki = 0.05;
        # float Kd = 0.1;

        # GUNNAR
        # self.tank_motor_pid = PidController(0.0, kP=2.0, kI=0.1, kD=0.75)
        # self.tank_motor_pid = PidController(0.0, kP=4.0, kI=0.2, kD=1.2)
        self.tank_motor_pid = PidController(0.0, kP=8.0, kI=0.5, kD=2.5, output_min=-1.0, output_max=1.0)

        # DAVID
        # self.tank_motor_pid = PidController(0.0, kP=1.0, kI=0.2, kD=0.75)
        # self.tank_motor_pid = PidController(0.0, kP=0.0, kI=0.3, kD=1.2)
        # self.tank_motor_pid = PidController(0.0, kP=0.0, kI=0.35, kD=2.5)

        # ADAM
        # self.tank_motor_pid = PidController(0.0, kP=4.0, kI=0.0, kD=1.2)
        # self.tank_motor_pid = PidController(0.0, kP=8.0, kI=0.0, kD=2.0)

        self._next_value_log_counter = 0  # For debouncing next_value log

        cmd_topic = str(self.get_parameter(CMD_TOPIC_PARAM).get_parameter_value().string_value)
        status_topic = str(self.get_parameter(STATUS_TOPIC_PARAM).get_parameter_value().string_value)
        if not cmd_topic or not status_topic:
            raise TypeError("Missing topic arguments ({}, {})".format(CMD_TOPIC_PARAM, STATUS_TOPIC_PARAM))

        self.level_cmd_subscription = self.create_subscription(Float32, cmd_topic, self.handle_tank_cmd, 10)
        self.publisher = self.create_publisher(TankStatus, status_topic, 10)

        self.tank = create_tank(
            simulate=should_simulate,
            motor_pin=motor_pin,
            direction_pin=direction_pin,
            floor=floor_value,
            ceiling=ceiling_value,
            channel=distance_sensor_channel,
        )

        self.check_target_updater = self.create_timer(1.0 / UPDATE_FREQUENCY, self.target_loop)

        self.get_logger().info(
            "{}Tank node started. Motor pin: {}, Direction pin: {}, "
            "Distance sensor channel: {}, Update frequency: {}, "
            "Range: {} - {} mm, CMD topic: {}, status topic: {}".format(
                "SIMULATE " if should_simulate else "",
                motor_pin,
                direction_pin,
                distance_sensor_channel,
                UPDATE_FREQUENCY,
                floor_value,
                ceiling_value,
                cmd_topic,
                status_topic,
            )
        )

    def stop_checking_against_target(self) -> None:
        self.target_level = None
        self.is_autocorrecting = False

    def handle_tank_cmd(self, msg: Float32) -> None:
        bounded_level = bounded_tank_level(msg.data)
        if bounded_level is None:
            self.get_logger().warning(f"Rejecting invalid tank target level {msg.data}")
            return
        requested_target_level = bounded_level
        target_level = clamp(
            requested_target_level,
            DEFAULT_TANK_TARGET_CONFIG.level_floor,
            DEFAULT_TANK_TARGET_CONFIG.level_ceiling,
        )

        self.target_status = "adjusting"
        self.target_level = target_level

        self.get_logger().info(f"Setting set point {self.target_level}")

        # NOTE: can we even do this here? since we're gonna call this often,
        # so the cumulative error is gonna be reset all the time.
        self.tank_motor_pid.reset_cumulative_error()

        reset_ramp = abs(self.target_level - self.tank_motor_pid.set_point) > 0.05
        if reset_ramp:
            self.clamp_value = 0.0

        self.tank_motor_pid.update_set_point(self.target_level)

    def publish_status(self, current_level: Optional[float]) -> None:
        if current_level is not None:
            msg = TankStatus()
            msg.current_level = float(current_level)
            msg.target_level = []
            if self.target_level is not None:
                msg.target_level.append(self.target_level)
            msg.is_autocorrecting = self.is_autocorrecting
            msg.target_status = self.target_status
            self.publisher.publish(msg)

    def target_loop(self) -> None:
        self.current_level = self.tank.get_level()

        self.running_average.add_sample(self.current_level)

        level_average = self.running_average.get_average()

        self.publish_status(self.current_level)

        if self.target_level is None:
            return

        stop_reason = tank_stop_reason(level_average, self.target_level)
        if stop_reason is not None:
            self.tank.stop()
            self.tank_motor_pid.reset_cumulative_error()
            self.target_status = stop_reason
        else:
            if self.clamp_value < self.clamp_max_value:
                self.clamp_value = min(self.clamp_value + 0.05, self.clamp_max_value)
            self.tank_motor_pid.update_output_limits(-self.clamp_value, self.clamp_value)
            pid_value = self.tank_motor_pid.compute(level_average)
            next_value = pid_value

            # Debounce log: only log once every 10 cycles
            self._next_value_log_counter += 1
            if self._next_value_log_counter >= 10:
                self.get_logger().info(f"{next_value=} {pid_value=}")
                self._next_value_log_counter = 0

            self.tank.run_motor(next_value)

    def shutdown(self) -> None:
        self.stop_checking_against_target()
        self.tank.shutdown()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TankNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
