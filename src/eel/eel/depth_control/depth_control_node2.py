from typing import List, Optional
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from ..utils.pid_controller import PidController
from std_msgs.msg import Float32
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
from ..utils.utils import clamp

# What we found for the real AUV
# FRONT_NEUTRAL = 0.45
# REAR_NEUTRAL = 0.69

# What is already hardcoded in simulation mode
FRONT_NEUTRAL = 0.5
REAR_NEUTRAL = 0.5

average_neutral = (FRONT_NEUTRAL + REAR_NEUTRAL) / 2  # = 0.57
differential_neutral = FRONT_NEUTRAL - REAR_NEUTRAL  # = -0.24

sim_depth_controller = PidController(
    set_point=0.0,
    kP=0.5,
    kI=0.0,  # 0.02,  # Kp/25 to eliminate steady-state error
    kD=0.01,  # 0.5 * 22 / 4 = 2.75
)

sim_pitch_controller = PidController(
    set_point=0.0,
    kP=0.02,
    kI=0.0,
    kD=0.01,
)


class DepthControlNode2(Node):
    def __init__(
        self,
        node_name: str = "depth_control2",
    ) -> None:
        super().__init__(node_name)
        self.depth_controller: Optional[PidController] = None
        self.pitch_controller: Optional[PidController] = None

        self.current_depth = 0.0
        self.current_pitch = 0.0

        self.create_subscription(
            DepthControlCmd, DEPTH_CONTROL_CMD, self.handle_cmd_msg, 10
        )

        self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_msg, 10)
        self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10
        )

        self.front_tank_pub = self.create_publisher(Float32, FRONT_TANK_CMD, 10)
        self.rear_tank_pub = self.create_publisher(Float32, REAR_TANK_CMD, 10)
        self.create_timer(0.1, self.loop)
        self.get_logger().info("Depth control [2] started.")

    def handle_pressure_msg(self, msg: PressureStatus):
        self.current_depth = msg.depth

    def handle_imu_msg(self, msg: ImuStatus):
        self.current_pitch = msg.pitch

    def handle_cmd_msg(self, msg: DepthControlCmd) -> None:
        self.depth_target = msg.depth_target
        self.pitch_target = msg.pitch_target

        self.pitch_controller = sim_pitch_controller
        self.depth_controller = sim_depth_controller

        self.depth_controller.update_set_point(msg.depth_target)
        self.pitch_controller.update_set_point(msg.pitch_target)

    def pub_front(self, value: float) -> None:
        clamped = clamp(value, 0.0, 1.0)
        msg = Float32()
        msg.data = clamped
        self.front_tank_pub.publish(msg)

    def pub_rear(self, value: float) -> None:
        clamped = clamp(value, 0.0, 1.0)
        msg = Float32()
        msg.data = clamped
        self.rear_tank_pub.publish(msg)

    def loop(self) -> None:
        if not (self.pitch_controller and self.depth_controller):
            return

        depth_output = self.depth_controller.compute(self.current_depth)
        average_level = average_neutral + depth_output

        pitch_output = self.pitch_controller.compute(self.current_pitch)
        differential = differential_neutral + pitch_output

        next_front_tank_level = average_level + differential / 2
        next_rear_tank_level = average_level - differential / 2

        self.pub_front(next_front_tank_level)
        self.pub_rear(next_rear_tank_level)


def main(args=None):
    rclpy.init(args=args)
    node = DepthControlNode2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
