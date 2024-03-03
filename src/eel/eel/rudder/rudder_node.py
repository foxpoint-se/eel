#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import sys
import math
from eel_interfaces.msg import ImuStatus

from ..utils.topics import (
    RUDDER_STATUS,
    RUDDER_X_CMD,
    RUDDER_Y_CMD,
    IMU_STATUS,
)
from ..utils.constants import SIMULATE_PARAM
from ..utils.utils import clamp
from .actuator.actuator import get_xy_rudder
from .actuator.types import Vector2d


def rotate_vector(vector: Vector2d, rotation_degrees: float) -> Vector2d:
    rotation_radians = math.radians(rotation_degrees)
    x = vector["x"]
    y = vector["y"]
    rotated_x = math.cos(rotation_radians) * x - math.sin(rotation_radians) * y
    rotated_y = math.sin(rotation_radians) * x + math.cos(rotation_radians) * y
    return {"x": rotated_x, "y": rotated_y}


class Rudder(Node):
    def __init__(self):
        super().__init__("rudder_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)

        pigpiod_host_parameter = "pigpiod_host"
        self.declare_parameter(pigpiod_host_parameter, "localhost")
        self.pigpiod_host = str(self.get_parameter(pigpiod_host_parameter).value)

        self.x_subscription = self.create_subscription(
            Float32, RUDDER_X_CMD, self.handle_x_cmd, 10
        )
        self.y_subscription = self.create_subscription(
            Float32, RUDDER_Y_CMD, self.handle_y_cmd, 10
        )
        self.rudder_status_publisher = self.create_publisher(Vector3, RUDDER_STATUS, 10)

        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self._handle_imu_msg, 10
        )

        self.current_x_cmd: float = float()
        self.current_y_cmd: float = float()

        self.current_roll = 0.0
        self.current_rudder_status: Vector2d = {"x": 0.0, "y": 0.0}

        self.xy_rudder = get_xy_rudder(
            {"simulate": self.should_simulate, "pigpiod_host": self.pigpiod_host}
        )

        self.get_logger().info(
            "{}Rudder node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def shutdown(self):
        self.get_logger().info("Rudder node shutting down...")
        self.xy_rudder.shutdown()

    def _handle_imu_msg(self, msg: ImuStatus):
        self.current_roll = msg.roll
        self.merge_and_handle_commands()

    def handle_x_cmd(self, msg: Float32) -> None:
        self.current_x_cmd = msg.data
        self.merge_and_handle_commands()

    def handle_y_cmd(self, msg: Float32) -> None:
        self.current_y_cmd = msg.data
        self.merge_and_handle_commands()

    def merge_and_handle_commands(self) -> None:
        direction: Vector2d = {
            "x": self.current_x_cmd,
            "y": self.current_y_cmd,
        }
        self.calc_and_send(rudder_direction=direction, roll_degrees=self.current_roll)

    def calc_and_send(self, rudder_direction: Vector2d, roll_degrees: float):
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


def main(args=None):
    rclpy.init(args=args)
    node = Rudder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
