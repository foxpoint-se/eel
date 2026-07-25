from typing import Optional, Protocol

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from rclpy.type_support import GetResultServiceResponse

from eel_interfaces.action import Dive
from eel_interfaces.action._dive import _Dive_Impl

DiveGoalHandle = ClientGoalHandle[Dive.Goal, Dive.Result, Dive.Feedback, _Dive_Impl]
SendGoalFuture = Future[DiveGoalHandle]
GetResultFuture = Future[GetResultServiceResponse[Dive.Result]]


class DiveFeedbackMessage(Protocol):
    feedback: Dive.Feedback


class DiveActionClient(Node):
    def __init__(self) -> None:
        super().__init__('dive_action_client')
        self._action_client = ActionClient(self, Dive, "dive")

        self.logger = self.get_logger()

    def send_goal(self, wanted_depth: float) -> None:
        goal_msg = Dive.Goal()
        goal_msg.wanted_depth = wanted_depth

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future: SendGoalFuture) -> None:
        goal_handle = future.result()
        if goal_handle is None:
            return

        if not goal_handle.accepted:
            self.logger.info("Goal was rejected.")
            return

        self.logger.info("Goal accepted.")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future: GetResultFuture) -> None:
        result_response = future.result()
        if result_response is None:
            return
        result = result_response.result
        self.logger.info(f"Result, final depth: {result.final_depth}m")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg: DiveFeedbackMessage) -> None:
        feedback = feedback_msg.feedback
        self.logger.info(f"Recieved feedback, now at: {round(feedback.current_depth, 5)}m")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    
    action_client = DiveActionClient()
    action_client.send_goal(2.0)
    
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
