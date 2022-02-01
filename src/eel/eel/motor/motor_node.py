#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


MOTOR_TOPIC = "motor"
SIMULATE_PARAM = "simulate"
FAKE_EEL_MOTOR_TOPIC = "/fake_eel_motor"


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value


class Motor(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.motor_subscription = self.create_subscription(
            Float32, MOTOR_TOPIC, self.handle_motor_msg, 10
        )

        if self.should_simulate:
            self.fake_eel_linear_publisher = self.create_publisher(
                Float32, FAKE_EEL_MOTOR_TOPIC, 10
            )
        else:
            from .motor_control import MotorControl

            self.motor_control = MotorControl()

        self.get_logger().info(
            "{}Motor node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def handle_motor_msg(self, msg):
        motor_value = clamp(msg.data, -1, 1)

        if not self.should_simulate:
            if motor_value == 0:
                self.motor_control.stop()
            elif motor_value > 0:
                self.motor_control.forward(speed=motor_value * 100)
            elif motor_value < 0:
                self.motor_control.backward(speed=abs(motor_value) * 100)
        else:
            fake_eel_msg = Float32()
            fake_eel_msg.data = motor_value
            self.fake_eel_linear_publisher.publish(fake_eel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
