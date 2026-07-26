from enum import IntEnum
from typing import Any, Callable

from rclpy.node import Node
from rclpy.task import Future

class GoalResponse(IntEnum):
    ACCEPT = 1
    REJECT = 2

class CancelResponse(IntEnum):
    ACCEPT = 1
    REJECT = 2

class ActionClient:
    def __init__(
        self,
        node: Node,
        action_type: object,
        action_name: str,
        *,
        callback_group: object | None = ...,
        goal_service_qos_profile: object = ...,
        result_service_qos_profile: object = ...,
        cancel_service_qos_profile: object = ...,
        feedback_sub_qos_profile: object = ...,
        status_sub_qos_profile: object = ...,
    ) -> None: ...
    def wait_for_server(self, timeout_sec: float | None = ...) -> bool: ...
    def send_goal_async(
        self,
        goal: object,
        feedback_callback: Callable[..., None] | None = ...,
        goal_uuid: object | None = ...,
    ) -> Future[Any]: ...

class ActionServer:
    def __init__(
        self,
        node: Node,
        action_type: object,
        action_name: str,
        execute_callback: Callable[..., object],
        *,
        callback_group: object | None = ...,
        goal_callback: Callable[..., object] = ...,
        handle_accepted_callback: Callable[..., object] = ...,
        cancel_callback: Callable[..., object] = ...,
        goal_service_qos_profile: object = ...,
        result_service_qos_profile: object = ...,
        cancel_service_qos_profile: object = ...,
        feedback_pub_qos_profile: object = ...,
        status_pub_qos_profile: object = ...,
    ) -> None: ...
