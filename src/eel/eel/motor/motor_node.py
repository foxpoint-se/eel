#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .motor_sim import MotorSimulator


MOTOR_TOPIC = "motor"
SIMULATE_PARAM = "simulate"


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
            simulator = MotorSimulator()
            self.stop = simulator.stop
            self.forward = simulator.forward
            self.backward = simulator.backward
        else:
            from .motor_control import MotorControl

            motor_control = MotorControl()
            self.stop = motor_control.stop
            self.forward = motor_control.forward
            self.backward = motor_control.backward

        self.get_logger().info(
            "{}Motor node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def handle_motor_msg(self, msg):
        motor_value = clamp(msg.data, -1, 1)
        if motor_value == 0:
            self.stop()
        elif motor_value > 0:
            self.forward(speed=motor_value * 100)
        elif motor_value < 0:
            self.backward(speed=abs(motor_value) * 100)


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
