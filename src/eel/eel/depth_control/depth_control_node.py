#!/usr/bin/env python3
from ..utils.pid_tuning import get_production_pid_settings
from ..utils.pid_controller import PidController
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


class DepthControlNode(Node):
    def __init__(self):
        super().__init__("depth_control_node")

        self.should_control_depth = False
        self.should_control_pitch = False
        self.depth_target = None
        self.target_pitch = None
        self.current_depth = 0.0
        self.current_pitch = 0.0
        self.last_depth_at = None
        self.last_depth = None
        self.current_front_tank_level = None
        self.current_rear_tank_level = None

        # TODO: conditionally set pid settings, based on what we find when tuning hardware
        # depth_Ku, depth_Tu, pitch_Ku, pitch_Tu = get_simulation_pid_settings()
        depth_Ku, depth_Tu, pitch_Ku, pitch_Tu = get_production_pid_settings()
        self.depth_Ku = depth_Ku
        self.depth_Tu = depth_Tu
        self.pitch_Ku = pitch_Ku
        self.pitch_Tu = pitch_Tu

        self.pitch_pid_controller = None
        self.depth_pid_controller = None
        self.front_controller = None
        self.rear_controller = None

        self.pid_controller = None

        self.pid_lib_controller = None

        self.publisher = self.create_publisher(
            DepthControlStatus, DEPTH_CONTROL_STATUS, 10
        )

        # TODO: enable these conditionally, for use when tuning PID
        self.pid_publisher = self.create_publisher(Float32, "pid_error", 10)
        self.pid_publisher_base = self.create_publisher(Float32, "pid_error_target", 10)
        self.front_tank_depth_publisher = self.create_publisher(
            Float32, "front_depth_pid", 10
        )
        self.front_tank_pitch_publisher = self.create_publisher(
            Float32, "front_pitch_pid", 10
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

        self.last_depth_controller_output = None
        self.last_pitch_controller_output = None

    def handle_cmd_msg(self, msg):
        self.depth_target = msg.depth_target
        self.pitch_target = msg.pitch_target

        # TODO: Think about how to handle fluctuating sensor values. The noise from them will result in constant updates from the PIDs.
        # Maybe kill the PIDs when inside an accepted target range and re-initialize them when the vehicle has drifted away.
        # We could also make the simulators return fluctuating values, so that we can code the logic even in simulation mode.

        depth_Ku = self.depth_Ku
        depth_Tu = self.depth_Tu

        # depth_Kp, depth_Ki, depth_Kd = lookup_zieglernichols_gains(
        #     depth_Ku, depth_Tu, msg.depth_pid_type
        # )
        depth_Kp = 0.25
        depth_Ki = 0.0
        depth_Kd = 0.0

        self.logger.info(
            "init depth pid with Kp {} Ki {} Kd {}".format(depth_Kp, depth_Ki, depth_Kd)
        )
        self.depth_pid_controller = PidController(
            self.depth_target,
            depth_Kp,
            depth_Ki,
            depth_Kd,
        )

        pitch_Ku = self.pitch_Ku
        pitch_Tu = self.pitch_Tu

        # pitch_Kp, pitch_Ki, pitch_Kd = lookup_zieglernichols_gains(
        #     pitch_Ku, pitch_Tu, msg.pitch_pid_type
        # )

        pitch_Kp = 0.02
        pitch_Ki = 0.0
        # pitch_Kd = 0.127
        pitch_Kd = 0.127

        self.logger.info(
            "init pitch pid with Kp {} Ki {} Kd {}".format(pitch_Kp, pitch_Ki, pitch_Kd)
        )

        self.pitch_pid_controller = PidController(
            self.pitch_target,
            pitch_Kp,
            pitch_Ki,
            pitch_Kd,
        )

    def handle_imu_msg(self, msg):
        self.current_pitch = msg.pitch

    def handle_front_tank_msg(self, msg):
        self.current_front_tank_level = msg.current_level

    def handle_rear_tank_msg(self, msg):
        self.current_rear_tank_level = msg.current_level

    def handle_pressure_msg(self, msg):
        self.current_depth = msg.depth

    def log_pid_error(self, pid_error):
        msg = Float32()
        msg.data = pid_error
        self.pid_publisher.publish(msg)
        base_msg = Float32()
        base_msg.data = 0.0
        self.pid_publisher_base.publish(base_msg)

    def loop(self):
        if self.pitch_pid_controller and self.depth_pid_controller:
            # if self.last_pitch_controller_output and abs(self.pitch_target - self.current_pitch) <= 1.5:
            #     pitch_controller_output = self.last_pitch_controller_output
            # else:
            pitch_controller_output = self.pitch_pid_controller.compute(
                self.current_pitch
            )

            # self.last_pitch_controller_output = pitch_controller_output

            pitch_front_tank = pitch_controller_output
            pitch_rear_tank = -pitch_controller_output

            # if self.last_depth_controller_output and abs(self.depth_target - self.current_depth) <= 0.025:
            #     depth_controller_output = self.last_depth_controller_output
            # else:
            depth_controller_output = self.depth_pid_controller.compute(
                self.current_depth
            )

            # self.last_depth_controller_output = depth_controller_output

            next_front_tank_level = (0.2 * pitch_front_tank) + (
                0.8 * depth_controller_output
            )
            # next_front_tank_level = depth_controller_output

            next_front_tank_level = 0.67 * next_front_tank_level

            next_front_tank_level = next_front_tank_level + 0.45

            next_rear_tank_level = (0.2 * pitch_rear_tank) + (
                0.8 * depth_controller_output
            )
            # next_rear_tank_level = depth_controller_output

            next_rear_tank_level = 1.33 * next_rear_tank_level

            next_rear_tank_level = next_rear_tank_level + 0.69

            # self.log_pid_error(self.pitch_target - self.current_pitch)

            self.publish_debug_values(depth_controller_output, pitch_front_tank)

            front_msg = Float32()
            front_msg.data = next_front_tank_level
            rear_msg = Float32()
            rear_msg.data = next_rear_tank_level

            self.front_tank_pub.publish(front_msg)
            self.rear_tank_pub.publish(rear_msg)

            # TODO add to status: current depth velocity, current target depth
            # TODO add to status: current pitch velocity, current target pitch
            # TODO add to status: status --> going to target, target reached etc
            # status_msg = DepthControlStatus()
            # status_msg.is_adjusting_depth = self.should_control_depth
            # status_msg.is_adjusting_pitch = self.should_control_pitch
            # self.publisher.publish(status_msg)

    def publish_debug_values(self, depth_front_tank, pitch_front_tank):
        depth_front_msg = Float32()
        depth_front_msg.data = depth_front_tank
        pitch_front_msg = Float32()
        pitch_front_msg.data = pitch_front_tank

        self.front_tank_depth_publisher.publish(depth_front_msg)
        self.front_tank_pitch_publisher.publish(pitch_front_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
