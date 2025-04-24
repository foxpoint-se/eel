#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import math
import sys
import signal
from enum import Enum
from eel_interfaces.msg import ImuStatus

from ..utils.topics import (
    RUDDER_STATUS,
    RUDDER_X_CMD,
    RUDDER_X_SET_OFFSET,
    RUDDER_X_OFFSET,
    RUDDER_Y_CMD,
    RUDDER_Y_SET_OFFSET,
    RUDDER_Y_OFFSET,
    IMU_STATUS,
)
from ..utils.constants import SIMULATE_PARAM
from ..utils.utils import clamp
from .actuator.actuator import get_xy_rudder
from .actuator.types import Vector2d


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
    def __init__(self):
        super().__init__("rudder_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)
        self.logger = self.get_logger()

        pigpiod_host_parameter = "pigpiod_host"
        self.declare_parameter(pigpiod_host_parameter, "localhost")
        self.pigpiod_host = str(self.get_parameter(pigpiod_host_parameter).value)

        self.x_subscription = self.create_subscription(
            Float32, RUDDER_X_CMD, self.handle_x_cmd, 10
        )
        self.x_offset_subscription = self.create_subscription(
            Float32, RUDDER_X_SET_OFFSET, self.handle_x_offset, 10
        )
        self.x_offset_publisher = self.create_publisher(Float32, RUDDER_X_OFFSET, 10)
        self.y_subscription = self.create_subscription(
            Float32, RUDDER_Y_CMD, self.handle_y_cmd, 10
        )
        self.y_offset_subscription = self.create_subscription(
            Float32, RUDDER_Y_SET_OFFSET, self.handle_y_offset, 10
        )
        self.y_offset_publisher = self.create_publisher(Float32, RUDDER_Y_OFFSET, 10)
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

        self.logger.info(
            "{}Rudder node started.".format("SIMULATE " if self.should_simulate else "")
        )

        # Send the initial offset values for rudder x and y for front end to pick up
        self.publish_rudder_offset_value(Rudders.RUDDER_X)
        self.publish_rudder_offset_value(Rudders.RUDDER_Y)

    def shutdown(self):
        self.logger.info("Rudder node shutting down...")
        self.xy_rudder.shutdown()

    def _handle_imu_msg(self, msg: ImuStatus):
        self.current_roll = msg.roll
        self.merge_and_handle_commands()

    def handle_x_cmd(self, msg: Float32) -> None:
        self.current_x_cmd = msg.data
        self.merge_and_handle_commands()

    def publish_rudder_offset_value(self, rudder_type) -> None:
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

    def handle_x_offset(self, msg:Float32) -> None:
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
        self.current_y_cmd = msg.data
        self.merge_and_handle_commands()
    
    def handle_y_offset(self, msg:Float32) -> None:
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


def shutdown_handler(signum, frame, node: Rudder):
    # Call the shutdown method of your node
    node.shutdown()
    rclpy.try_shutdown()
    sys.exit(0)


# Register the shutdown handler for SIGTERM signal


def main(args=None):
    rclpy.init(args=args)
    node = Rudder()
    signal.signal(
        signal.SIGTERM, lambda signum, frame: shutdown_handler(signum, frame, node)
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # pass
        sys.exit(1)
    # except ExternalShutdownException:
    #     sys.exit(1)
    finally:
        node.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
