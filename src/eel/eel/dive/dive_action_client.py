import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from eel_interfaces.action import Dive
from eel_interfaces.srv import StartDive, CancelDive
from eel_interfaces.msg import DiveStatus

from ..utils.topics import DIVE_STATUS


MAX_SRV_RESPONSE_TIME = 3


class DiveActionClient(Node):
    def __init__(self):
        super().__init__("dive_action_client")
        self.logger = self.get_logger()
        self._action_client = ActionClient(self, Dive, "dive")
        self._goal_handle = None
        self.dive_feedback_counter = 0

        self.start_dive_srv = self.create_service(StartDive, "start_dive", self.start_dive)
        self.cancel_dive_srv = self.create_service(CancelDive, "cancel_dive", self.cancel_dive)

        self.status_publisher = self.create_publisher(DiveStatus, DIVE_STATUS, 10)

        self.logger.info("Dive action client node started.")

    def start_dive(self, request, response):
        request_recieved = time.time()
        wanted_depth = request.wanted_depth
        dive_time = request.dive_time

        # Check if there already is a goal in progress
        if self._goal_handle is not None:
            status = self._goal_handle.status()
            self.logger.info(f"Currently goal running with status {status}, unable to set new goal.")
            response.dive_accepted = False
            
            return response

        self.send_goal(wanted_depth, dive_time)
        response.dive_accepted = True

        return response

    def cancel_dive(self, request, response):
        if self._goal_handle is None:
            self.logger.info("No active goal handle, cannot issue cancel command.")
            response.dive_canceled = False

            return response

        self.cancel_goal()
        response.dive_canceled = True

        return response

    def send_goal(self, wanted_depth, dive_time):
        goal_msg = Dive.Goal()
        goal_msg.wanted_depth = wanted_depth
        goal_msg.dive_time = dive_time

        self._action_client.wait_for_server()

        self.logger.info(f"Initiating dive to depth {wanted_depth} for {dive_time}s.")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def cancel_goal(self):
        self._goal_handle.cancel_goal_async()
    
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.logger.info("Goal was rejected by server.")
            return

        self.logger.info("Goal was accepted!!!")
        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            result = future.result().result
            self.logger.info(f"Goal finished successfully, action time: {result.action_time}")

        elif status == GoalStatus.STATUS_CANCELED:
            self.logger.info("Goal was cancelled")

        self._goal_handle = None
        self.dive_feedback_counter = 0

    def feedback_callback(self, feedback_msg):
        self.dive_feedback_counter += 1

        if self.dive_feedback_counter % 5 == 0:
            msg = DiveStatus()
            msg.current_depth = feedback_msg.feedback.current_depth
            msg.time_left = feedback_msg.feedback.time_left
            msg.depth_reached_after =  feedback_msg.feedback.depth_reached_after

            self.status_publisher.publish(msg)

        feedback = feedback_msg.feedback
        self.logger.info(f"Recieved feedback, now at: {round(feedback.current_depth, 5)}m")

def main(args=None):
    rclpy.init(args=args)
    action_client = DiveActionClient()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
