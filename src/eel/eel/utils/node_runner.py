from collections.abc import Callable

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


def spin_node_until_shutdown(
    node: Node,
    *,
    executor: MultiThreadedExecutor | None = None,
    cleanup: Callable[[], None] | None = None,
) -> None:
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if cleanup is not None:
            try:
                cleanup()
            except Exception:
                pass
        node.destroy_node()
        rclpy.try_shutdown()
