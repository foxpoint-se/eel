"""Integration boot-test fixture: one-shot dive action client."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Protocol, TypeAlias

import rclpy
from eel.utils.node_runner import spin_node_until_shutdown
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from eel_interfaces.action import Dive

if TYPE_CHECKING:
    from rclpy.type_support import GetResultServiceResponse

    DiveGoalHandle: TypeAlias = ClientGoalHandle[Dive.Goal, Dive.Result, Dive.Feedback, object]
    SendGoalFuture: TypeAlias = Future[DiveGoalHandle]
    GetResultFuture: TypeAlias = Future[GetResultServiceResponse[Dive.Result]]
else:
    DiveGoalHandle = ClientGoalHandle
    SendGoalFuture = Future
    GetResultFuture = Future


class DiveFeedbackMessage(Protocol):
    feedback: Dive.Feedback


class DiveDemo(Node):
    def __init__(self) -> None:
        super().__init__("dive_demo")
        self._action_client = ActionClient(self, Dive, "dive")
        self.logger = self.get_logger()

    def send_goal(self, wanted_depth: float, dive_time: float = 10.0) -> None:
        goal_msg = Dive.Goal()
        goal_msg.wanted_depth = wanted_depth
        goal_msg.dive_time = dive_time

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: SendGoalFuture) -> None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.logger.info("Goal was rejected.")
            return

        self.logger.info("Goal accepted.")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: GetResultFuture) -> None:
        result_response = future.result()
        result = result_response.result
        self.logger.info(f"Result, final depth: {result.final_depth}m")
        rclpy.try_shutdown()

    def feedback_callback(self, feedback_msg: DiveFeedbackMessage) -> None:
        feedback = feedback_msg.feedback
        self.logger.info(f"Received feedback, now at: {round(feedback.current_depth, 5)}m")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    demo = DiveDemo()
    demo.send_goal(2.0)
    spin_node_until_shutdown(demo)


if __name__ == "__main__":
    main()
