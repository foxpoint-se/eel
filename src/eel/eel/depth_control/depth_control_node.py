#!/usr/bin/env python3
from ..utils.utils import clamp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from time import time
from eel_interfaces.msg import (
    DepthControlStatus,
    DepthControlCmd,
    ImuStatus,
    PressureStatus,
    TankStatus,
)
from ..utils.topics import (
    DEPTH_CONTROL_STATUS,
    DEPTH_CONTROL_CMD,
    IMU_STATUS,
    PRESSURE_STATUS,
    FRONT_TANK_CMD,
    REAR_TANK_CMD,
    FRONT_TANK_STATUS,
    REAR_TANK_STATUS,
)

UPDATE_FREQUENCY = 5

DEPTH_TOLERANCE_METERS = 0.2

DEPTH_DIFF_MAX_OUT = 2.0

TERMINAL_VELOCITY_MPS = 0.5


def get_sign(value):
    if value >= 0:
        return 1
    else:
        return -1


def get_are_same_sign(value_1, value_2):
    if get_sign(value_1) == get_sign(value_2):
        return True
    return False


def is_within_accepted_target_boundaries(current_level, target_level, tolerance):
    low_threshold = target_level - (tolerance / 2)
    high_threshold = target_level + (tolerance / 2)
    is_within_target = low_threshold <= current_level <= high_threshold
    return is_within_target


class DepthControlNode(Node):
    def __init__(self):
        super().__init__("depth_control_node")

        self.should_control_depth = False
        self.should_control_pitch = False
        self.depth_target = None
        self.target_pitch = None
        self.current_depth = None
        self.current_pitch = None
        self.last_depth_at = None
        self.last_depth = None
        self.current_front_tank_level = None
        self.current_rear_tank_level = None

        self.publisher = self.create_publisher(
            DepthControlStatus, DEPTH_CONTROL_STATUS, 10
        )
        self.create_subscription(
            DepthControlCmd, DEPTH_CONTROL_CMD, self.handle_cmd_msg, 10
        )

        self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_msg, 10)
        self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10
        )

        self.front_tank_pub = self.create_publisher(Float32, FRONT_TANK_CMD, 10)
        self.rear_tank_pub = self.create_publisher(Float32, REAR_TANK_CMD, 10)
        self.create_subscription(
            TankStatus, FRONT_TANK_STATUS, self.handle_front_tank_msg, 10
        )
        self.create_subscription(
            TankStatus, REAR_TANK_STATUS, self.handle_rear_tank_msg, 10
        )

        self.updater = self.create_timer(1.0 / UPDATE_FREQUENCY, self.loop)

        self.logger = self.get_logger()

        self.logger.info("Depth control node started.")

        self.cumulative_error = 0.0
        self.elapsed_time = 0.0
        self.last_error = 0.0

    def handle_cmd_msg(self, msg):
        self.depth_target = msg.depth_target
        self.pitch_target = msg.pitch_target
        self.should_control_depth = True
        self.should_control_pitch = False  # False for now

    def handle_imu_msg(self, msg):
        self.current_pitch = msg.pitch

    def handle_front_tank_msg(self, msg):
        self.current_front_tank_level = msg.current_level

    def handle_rear_tank_msg(self, msg):
        self.current_rear_tank_level = msg.current_level

    def handle_pressure_msg(self, msg):
        self.current_depth = msg.depth

    def control_depth(self, target, current, time_passed, distance_traveled):
        # P
        # 0.5 is pretty good
        # with only proportional term --> constant error, aka "steady state"
        proportional_gain = 0.5

        error = target - current
        proportional_term = proportional_gain * error  # -3.0, 2.0

        # I
        # 0.01 is pretty good
        integral_gain = 0.01
        self.cumulative_error += error * time_passed

        integral_term = self.cumulative_error * integral_gain

        # D
        # 3.0 is pretty good
        # derivative_gain = 5.0
        derivative_gain = 3.0

        error_delta = error - self.last_error
        time_delta = time_passed
        rate_of_error = error_delta / time_delta

        derivative_term = rate_of_error * derivative_gain

        proposed_controller_output = proportional_term + integral_term + derivative_term

        controller_output = proposed_controller_output

        # should possibly be slightly less than actuator limit
        # saturation_limit = 0.9
        # saturation_limit = 1.0

        # clamped_output = clamp(
        #     proposed_controller_output, -saturation_limit, saturation_limit
        # )
        # did_clamp = proposed_controller_output != clamped_output
        # is_in_saturation = did_clamp
        # are_same_sign = get_are_same_sign(proposed_controller_output, error)
        # when clamping, we should ignore integral term
        # should_clamp = are_same_sign and is_in_saturation
        # if should_clamp:
        #     controller_output = proportional_term + derivative_term
        # else:
        #     controller_output = proposed_controller_output

        # controller_output = proportional_term + derivative_term
        # if get_are_same_sign(integral_term, error) and not is_in_saturation:
        #     controller_output += integral_term

        # maybe we shouldn't adjust if within target and velocity is low?
        # if not is_within_accepted_target_boundaries(
        #     current, target, DEPTH_TOLERANCE_METERS
        # ):

        next_tank_level = controller_output
        self.logger.info(
            "actual {}, error {}, P {}, I {}, D {}".format(
                round(next_tank_level, 3),
                round(error, 3),
                round(proportional_term, 3),
                round(integral_term, 3),
                round(derivative_term, 5),
            )
        )

        self.last_error = error

        msg = Float32()
        msg.data = next_tank_level

        self.front_tank_pub.publish(msg)
        self.rear_tank_pub.publish(msg)

    def loop(self):
        now = time()
        if (
            self.depth_target is not None
            and self.should_control_depth
            and self.current_depth is not None
            and self.last_depth_at is not None
            and self.last_depth is not None
        ):
            time_passed = now - self.last_depth_at
            distance_traveled = self.current_depth - self.last_depth
            self.control_depth(
                self.depth_target, self.current_depth, time_passed, distance_traveled
            )

        self.last_depth_at = now
        self.last_depth = self.current_depth

        # TODO add to status: current depth velocity, current target depth
        # TODO add to status: current pitch velocity, current target pitch
        # TODO add to status: status --> going to target, target reached etc
        status_msg = DepthControlStatus()
        status_msg.is_adjusting_depth = self.should_control_depth
        status_msg.is_adjusting_pitch = self.should_control_pitch
        self.publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
