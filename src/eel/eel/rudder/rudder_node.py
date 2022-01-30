#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory


RUDDER_TOPIC = "rudder"


class Rudder(Node):
    def __init__(self):
        super().__init__("rudder_node")

        self.rudder_subscription = self.create_subscription(
            Float32, RUDDER_TOPIC, self.handle_rudder_msg, 10
        )

        factory = PiGPIOFactory()
        self.servo = Servo(
            13, pin_factory=factory, min_pulse_width=0.81 / 1000, max_pulse_width=2.2 / 1000
        )
        # self.servo.detach()

        self.get_logger().info("Rudder node started.")

    # def __del__(self):
    #     self.servo.detach()
    #     print('Destructor called, Employee deleted.')

    def shutdown(self):
        # self.get_logger().info("Rudder node shutting down...")
        self.servo.detach()
        # sleep(0.1)
        # self.servo.detach()
        # sleep(0.1)
        # self.servo.detach()

    def handle_rudder_msg(self, msg):
        self.get_logger().info("RUDDER NODE SAYS {}".format(msg))

        # new_value = msg.data
        # if new_value >= -1 and new_value <= 1:
        #     servo.value = new_value


def main(args=None):
    rclpy.init(args=args)
    node = Rudder()
    rclpy.get_default_context().on_shutdown(node.shutdown)
    
    # rclpy.spin(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.shutdown()


    # rclpy.shutdown()


if __name__ == "__main__":
    main()
