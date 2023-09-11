#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import sys
import math
from eel_interfaces.msg import ImuStatus
from .general_servo import RudderServo
from .rudder_sim import RudderSimulator
from ..utils.topics import (
    RUDDER_HORIZONTAL_CMD,
    RUDDER_HORIZONTAL_STATUS,
    RUDDER_VERTICAL_CMD,
    RUDDER_VERTICAL_STATUS,
    IMU_STATUS,
)
from ..utils.constants import SIMULATE_PARAM
from ..utils.utils import clamp


SIM_CALIBRATION_TERM = 0.0
CALIBRATION_TERM = 0.0


class Rudder(Node):
    def __init__(self):
        super().__init__("rudder_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.rudder_calibration_term = (
            SIM_CALIBRATION_TERM if self.should_simulate else CALIBRATION_TERM
        )
        # self.rudder_cmd_subscription = self.create_subscription(
        #     Float32, RUDDER_CMD, self.handle_rudder_msg, 10
        # )
        self.horizontal_cmd_subscription = self.create_subscription(
            Float32, RUDDER_HORIZONTAL_CMD, self.handle_horizontal_msg, 10
        )
        self.vertical_cmd_subscription = self.create_subscription(
            Float32, RUDDER_VERTICAL_CMD, self.handle_vertical_msg, 10
        )
        self.horizontal_status_publisher = self.create_publisher(
            Float32, RUDDER_HORIZONTAL_STATUS, 10
        )
        self.vertical_status_publisher = self.create_publisher(
            Float32, RUDDER_VERTICAL_STATUS, 10
        )

        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self._handle_imu_msg, 10
        )

        self.current_roll = 0.0
        self.current_vertical_control = 0.0
        self.current_horizontal_control = 0.0

        if self.should_simulate:
            # simulator = RudderSimulator()
            # self.detach = simulator.detach
            # self.set_value = simulator.set_value
            raise Exception("not implemented")
        if not self.should_simulate:
            horizontal_servo = RudderServo(
                pin=13,
                min_pulse_width=0.81 / 1000,
                max_pulse_width=2.2 / 1000,
                flip_direction=True,
                cap_min=-0.75,
                cap_max=0.75,
            )
            self.horizontal_detach = horizontal_servo.detach
            self.horizontal_set_value = horizontal_servo.set_value

            vertical_servo = RudderServo(
                pin=19,
                min_pulse_width=0.81 / 1000,
                max_pulse_width=2.2 / 1000,
                flip_direction=False,
                cap_min=-0.75,
                cap_max=0.75,
            )
            self.vertical_detach = vertical_servo.detach
            self.vertical_set_value = vertical_servo.set_value

        self.get_logger().info(
            "{}Rudder node started.".format(
                "SIMULATE " if self.should_simulate else ""
            )
        )

    def shutdown(self):
        self.get_logger().info("Rudder node shutting down...")
        self.horizontal_detach()
        self.vertical_detach()

    def _handle_imu_msg(self, msg):
        self.current_roll = msg.roll

        # TODO: THIS SHOULD NOT BE COMMENTED
        # self._set_rudder(self.current_vertical_control, self.current_horizontal_control, self.current_roll)
        # self.get_logger().info(f"BAJS {self.current_roll=}")
        self.calc_and_send(
            self.current_horizontal_control,
            self.current_vertical_control,
            self.current_roll,
        )

    def _set_rudder_VERTICAL(
        self, vertical_control, horizontal_control, roll_degrees
    ):
        roll_radians = math.radians(roll_degrees)

        # cos vinkeln gonger langden pa vektorn
        eel_vertical_part = math.cos(roll_radians) * vertical_control

        # a = math.sin(delta_lat_rad / 2.0) * math.sin(delta_lat_rad / 2.0) + math.cos(
        # sin vinklen gonger langden pa vektorn
        eel_horizontal_part = math.sin(roll_radians) * vertical_control

        # self.get_logger().info(f"REQ {round(vertical_control, 2)} ROLL {round(self.current_roll, 2)} VERT COMP {round(eel_vertical_part, 2)} HOR COMP {round(eel_horizontal_part, 2)}")

        self.vertical_set_value(eel_vertical_part)
        self.horizontal_set_value(eel_horizontal_part)

    def _set_rudder(self, vertical_control, horizontal_control, roll_degrees):
        # NOTE: for some reason it works when negative roll angle. for horizontal turns
        roll_radians = math.radians(-roll_degrees)
        eel_vertical_part = horizontal_control * math.sin(roll_radians)
        eel_horizontal_part = horizontal_control * math.cos(roll_radians)

        self.horizontal_set_value(eel_horizontal_part)
        self.vertical_set_value(eel_vertical_part)

    def handle_horizontal_msg(self, msg):
        new_horizontal = clamp(msg.data, -1, 1)
        self.current_horizontal_control = new_horizontal
        status_msg = Float32()
        status_msg.data = float(new_horizontal)
        self.horizontal_status_publisher.publish(status_msg)

        # vi har x1, det ar utslag i x-led
        # vi har y1, det ar utslag i y-led
        # det resulterar i en vektor x1, y1
        # iom att aalen roterar med vinkel B
        # saa maaste vi rotera vektor x1, y1 med negativ B

        # nya vektorn x2, y2 aer lika med
        # x2 = cos (-B) * x1 - sin (-B) * y1
        # y2 = sin (-B) * x1 + cos (-B) * y1

        self.calc_and_send(
            new_horizontal, self.current_vertical_control, self.current_roll
        )

    def calc_and_send(self, x, y, roll_degrees):
        roll_radians = math.radians(roll_degrees)
        rotation = -roll_radians
        x2 = math.cos(rotation) * x - math.sin(rotation) * y
        y2 = math.sin(rotation) * x + math.cos(rotation) * y

        self.horizontal_set_value(x2)
        self.vertical_set_value(y2)

    # TODO: IMU sometimes gives erroneous values. we could maybe filter out them somehow?
    # TODO: check that imu gives good roll when it should be level.

    def handle_horizontal_msg2(self, msg):
        # new_horizontal = msg.data + self.rudder_calibration_term
        new_horizontal = clamp(msg.data, -1, 1)

        # roll_radians = math.radians(-self.current_roll)
        # eel_vertical_part = new_horizontal * math.sin(roll_radians)
        # eel_horizontal_part = new_horizontal * math.cos(roll_radians)

        # self.horizontal_set_value(eel_horizontal_part)
        # self.vertical_set_value(eel_vertical_part)

        self._set_rudder(0.0, new_horizontal, self.current_roll)

        # self.get_logger().info(f"REQ {round(new_horizontal, 2)} ROLL {round(self.current_roll, 2)} VERT COMP {round(eel_vertical_part, 2)} HOR COMP {round(eel_horizontal_part, 2)}")

        self.current_horizontal_control = new_horizontal
        status_msg = Float32()
        status_msg.data = float(new_horizontal)
        self.horizontal_status_publisher.publish(status_msg)

    def handle_horizontal_msg2(self, msg):
        new_horizontal = msg.data + self.rudder_calibration_term
        new_horizontal = clamp(new_horizontal, -1, 1)
        self.current_horizontal_control = new_horizontal
        status_msg = Float32()
        status_msg.data = float(new_horizontal)
        self.horizontal_status_publisher.publish(status_msg)

    def handle_vertical_msg(self, msg):
        new_vertical = msg.data
        new_vertical = clamp(new_vertical, -1, 1)

        # roll_radians = math.radians(self.current_roll)
        # math.sin(math.radians(30)))

        # cos vinkeln gonger langden pa vektorn
        # eel_vertical_part = math.cos(roll_radians) * new_vertical

        # a = math.sin(delta_lat_rad / 2.0) * math.sin(delta_lat_rad / 2.0) + math.cos(
        # sin vinklen gonger langden pa vektorn
        # eel_horizontal_part = math.sin(roll_radians) * new_vertical

        # self.get_logger().info(f"REQ {round(new_vertical, 2)} ROLL {round(self.current_roll, 2)} VERT COMP {round(eel_vertical_part, 2)} HOR COMP {round(eel_horizontal_part, 2)}")

        # self.vertical_set_value(eel_vertical_part)
        # self.horizontal_set_value(eel_horizontal_part)

        self.current_vertical_control = new_vertical

        self.calc_and_send(
            self.current_horizontal_control, new_vertical, self.current_roll
        )

        # self._set_rudder(new_vertical, 0.0, self.current_roll)

        status_msg = Float32()
        status_msg.data = float(new_vertical)
        self.vertical_status_publisher.publish(status_msg)


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
