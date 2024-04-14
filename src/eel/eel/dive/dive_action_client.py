import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from eel_interfaces.action import Dive


class DiveActionClient(Node):
    def __init__(self):
        super().__init__('dive_action_client')
        self._action_client = ActionClient(self, Dive, "dive")

        self.logger = self.get_logger()

    def send_goal(self, wanted_depth):
        goal_msg = Dive.Goal()
        goal_msg.wanted_depth = wanted_depth

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.info("Goal was rejected.")
            return

        self.logger.info("Goal accepted.")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.logger.info(f"Result, final depth: {result.final_depth}m")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.logger.info(f"Recieved feedback, now at: {round(feedback.current_depth, 5)}m")

def main(args=None):
    rclpy.init(args=args)
    
    action_client = DiveActionClient()
    action_client.send_goal(2.0)
    
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
