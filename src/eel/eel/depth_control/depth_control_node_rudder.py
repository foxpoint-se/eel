#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import (
    DepthControlCmd,
    DepthControlStatus,
    ImuStatus,
    PressureStatus,
)

from ..utils.actuator_bounds import bounded_depth_target
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.pid_controller import PidController
from ..utils.topics import (
    DEPTH_CONTROL_CMD,
    DEPTH_CONTROL_STATUS,
    IMU_STATUS,
    PRESSURE_STATUS,
    RUDDER_Y_CMD,
)

UPDATE_FREQUENCY = 5


class DepthControlNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_control_node")
        self.logger = self.get_logger()
        self.logger.info("Depth control node started!!")

        self.depth_target = 0.0

        self.max_dive_angle = 30.0
        self.max_rudder_output = 1.0

        self.inner_pid_target_angle = PidController(
            0.0, kP=-35.0, kD=-10.0, output_min=-self.max_dive_angle, output_max=self.max_dive_angle
        )
        self.out_pid_rudder_output = PidController(
            0.0, kP=1 / 30, output_min=-self.max_rudder_output, output_max=self.max_rudder_output
        )

        self.current_pitch = 0.0
        self.current_depth = 0.0

        self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_msg, 10)
        self.create_subscription(PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10)
        self.create_subscription(DepthControlCmd, DEPTH_CONTROL_CMD, self.handle_cmd_msg, 10)

        self.rudder_publisher = self.create_publisher(Float32, RUDDER_Y_CMD, 10)
        self.status_publisher = self.create_publisher(DepthControlStatus, DEPTH_CONTROL_STATUS, 10)

        self.updater = self.create_timer(1.0 / UPDATE_FREQUENCY, self.compute_and_send)

    def compute_and_send(self) -> None:
        angle_pid_output = self.compute_new_target_angle()
        # self.logger.info(f"Current inner pid output = {angle_pid_output} degrees")

        rudder_pid_output = self.compute_new_rudder_output(angle_pid_output)
        # self.logger.info(f"Current outer pid output = {rudder_pid_output} rudder")

        rudder_msg = Float32()
        rudder_msg.data = float(rudder_pid_output)
        self.rudder_publisher.publish(rudder_msg)

        self.publish_status()

    def publish_status(self) -> None:
        status_msg = DepthControlStatus()
        status_msg.depth_target = self.depth_target
        self.status_publisher.publish(status_msg)

    def handle_imu_msg(self, msg: ImuStatus) -> None:
        self.current_pitch = msg.pitch

    def handle_pressure_msg(self, msg: PressureStatus) -> None:
        self.current_depth = msg.depth

    def handle_cmd_msg(self, msg: DepthControlCmd) -> None:
        depth_target = bounded_depth_target(msg.depth_target)
        if depth_target is None:
            self.logger.warning(f"Rejecting invalid depth control cmd depth={msg.depth_target}")
            return
        if depth_target != msg.depth_target:
            self.logger.warning(f"Clamped depth control cmd depth={msg.depth_target}->{depth_target}")
        self.depth_target = depth_target
        self.inner_pid_target_angle.update_set_point(depth_target)

    def compute_new_target_angle(self) -> float:
        pid_angle_output = self.inner_pid_target_angle.compute(self.current_depth)
        return -1 * pid_angle_output

    def compute_new_rudder_output(self, new_target_angle: float) -> float:
        self.out_pid_rudder_output.update_set_point(new_target_angle)
        return self.out_pid_rudder_output.compute(self.current_pitch)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DepthControlNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
