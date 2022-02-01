#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .motor_control import MotorControl


MOTOR_TOPIC = "motor"
SIMULATE_PARAM = "simulate"


class Motor(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.motor_subscription = self.create_subscription(
            Float32, MOTOR_TOPIC, self.handle_motor_msg, 10
        )

        # TODO: pass in self.should_simulate
        self.motor_control = MotorControl(simulate=True, logger=self.get_logger())

        self.get_logger().info(
            "{}Motor node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def handle_motor_msg(self, msg):
        self.get_logger().info("MOTOR NODE SAYS {}".format(msg))
        new_value = msg.data
        if new_value >= -1 and new_value <= 1:
            if new_value == 0:
                self.motor_control.stop()
            elif new_value > 0:
                self.motor_control.forward(speed=new_value * 100)
            elif new_value < 0:
                self.motor_control.backward(speed=abs(new_value) * 100)


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
