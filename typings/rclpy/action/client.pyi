from typing import Generic, TypeVar

from rclpy.task import Future

GoalT = TypeVar("GoalT")
ResultT = TypeVar("ResultT")
FeedbackT = TypeVar("FeedbackT")
ImplT = TypeVar("ImplT")

class ClientGoalHandle(Generic[GoalT, ResultT, FeedbackT, ImplT]):
    accepted: bool
    def get_result_async(self) -> Future[object]: ...
    def cancel_goal_async(self) -> Future[object]: ...
