#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ..utils.pid_controller import PidController
from std_msgs.msg import Float32

from eel_interfaces.msg import (
    ImuStatus,
    PressureStatus,
    DepthControlCmd,
    DepthControlStatus,
)
from ..utils.topics import (
    IMU_STATUS,
    PRESSURE_STATUS,
    RUDDER_VERTICAL_CMD,
    DEPTH_CONTROL_CMD,
    DEPTH_CONTROL_STATUS,
)


UPDATE_FREQUENCY = 5


class DepthControlNode(Node):
    def __init__(self):
        super().__init__("depth_control_node")
        self.logger = self.get_logger()
        self.logger.info("Depth control node started!!")

        self.depth_target = 0.0

        self.max_dive_angle = 30.0
        self.max_rudder_output = 1.0

        self.inner_pid_target_angle = PidController(0.0, kP=-12.5)
        self.out_pid_rudder_output = PidController(0.0, kP=1 / 30)

        self.current_pitch = 0.0
        self.current_depth = 0.0

        self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_msg, 10
        )
        self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10
        )
        self.create_subscription(
            DepthControlCmd, DEPTH_CONTROL_CMD, self.handle_cmd_msg, 10
        )

        self.rudder_publisher = self.create_publisher(
            Float32, RUDDER_VERTICAL_CMD, 10
        )
        self.status_publisher = self.create_publisher(
            DepthControlStatus, DEPTH_CONTROL_STATUS, 10
        )

        self.updater = self.create_timer(
            1.0 / UPDATE_FREQUENCY, self.compute_and_send
        )

    def compute_and_send(self):
        angle_pid_output = self.compute_new_target_angle()
        # self.logger.info(f"Current inner pid output = {angle_pid_output} degrees")

        rudder_pid_output = self.compute_new_rudder_output(angle_pid_output)
        # self.logger.info(f"Current outer pid output = {rudder_pid_output} rudder")

        rudder_msg = Float32()
        rudder_msg.data = float(rudder_pid_output)
        self.rudder_publisher.publish(rudder_msg)

        self.publish_status()

    def publish_status(self):
        status_msg = DepthControlStatus()
        status_msg.depth_target = self.depth_target
        self.status_publisher.publish(status_msg)

    def handle_imu_msg(self, msg):
        self.current_pitch = msg.pitch

    def handle_pressure_msg(self, msg):
        self.current_depth = msg.depth

    def handle_cmd_msg(self, msg):
        self.depth_target = msg.depth_target
        self.inner_pid_target_angle.update_set_point(msg.depth_target)

    def compute_new_target_angle(self):
        pid_angle_output = self.inner_pid_target_angle.compute(
            self.current_depth
        )

        if abs(pid_angle_output) > self.max_dive_angle:
            pid_angle_output = (
                self.max_dive_angle
                if pid_angle_output > 0
                else -1 * self.max_dive_angle
            )

        return -1 * pid_angle_output

    def compute_new_rudder_output(self, new_target_angle):
        self.out_pid_rudder_output.update_set_point(new_target_angle)
        rudder_output = self.out_pid_rudder_output.compute(self.current_pitch)

        if abs(rudder_output) > self.max_rudder_output:
            rudder_output = (
                self.max_rudder_output
                if rudder_output > 0
                else -1.0 * self.max_rudder_output
            )

        return rudder_output


def main(args=None):
    rclpy.init(args=args)
    node = DepthControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
