"""World plant ROS node.

Topic names are hardcoded to match today's eel stack (temporary).
When the plant edge is clearer, replace with remaps/params.
"""

from time import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import ImuStatus
from eel_world_sim.depth_model import DepthModel

# Temporary: same names as the live boat topics.
MOTOR_CMD_TOPIC = "motor/cmd"
IMU_STATUS_TOPIC = "imu/status"
WORLD_DEPTH_TOPIC = "world/depth"

PUBLISH_HZ = 10.0


class WorldSimNode(Node):
    def __init__(self) -> None:
        super().__init__("world_sim")
        self._depth_model = DepthModel()
        self._last_step_at = time()

        self.create_subscription(Float32, MOTOR_CMD_TOPIC, self._on_motor_cmd, 10)
        self.create_subscription(ImuStatus, IMU_STATUS_TOPIC, self._on_imu_status, 10)
        self._depth_pub = self.create_publisher(Float32, WORLD_DEPTH_TOPIC, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._on_timer)

    def _on_motor_cmd(self, msg: Float32) -> None:
        self._depth_model.set_motor_cmd(float(msg.data))

    def _on_imu_status(self, msg: ImuStatus) -> None:
        self._depth_model.set_pitch_deg(float(msg.pitch))

    def _on_timer(self) -> None:
        now = time()
        dt_s = now - self._last_step_at
        self._last_step_at = now

        depth_m = self._depth_model.step(dt_s)
        out = Float32()
        out.data = depth_m
        self._depth_pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WorldSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
