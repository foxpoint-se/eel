import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from eel_interfaces.action import Dive


class DiveActionServer(Node):


    def __init__(self):
        super().__init__("dive_action_server")
        self._action_server = ActionServer(
            self,
            Dive,
            "dive",
            self.execute_callback
        )

        self.current_depth = 0.0
        self.valid_offset = 0.05
        self.dive_direction = 1
        self.incline_per_sec = 0.1

        self.logger = self.get_logger()
        self.logger.info("Dive action server started")
        
    
    def execute_callback(self, goal_handle):
        wanted_depth = goal_handle.request.wanted_depth
        if wanted_depth > self.current_depth:
            self.dive_direction = 1
        else:
            self.dive_direction = 0

        self.logger.info(f"Executing goal, dive to {wanted_depth}m")

        feedback_msg = Dive.Feedback()
        feedback_msg.current_depth = self.current_depth

        min_offset = wanted_depth - self.valid_offset
        max_offset = wanted_depth + self.valid_offset

        while not (min(min_offset, max_offset) < self.current_depth < max(min_offset, max_offset)):
            self.current_depth += self.dive_direction * self.incline_per_sec
            
            feedback_msg.current_depth = self.current_depth
            goal_handle.publish_feedback(feedback_msg)
            
            self.logger.debug(f"Depth now at {self.current_depth}m")
            time.sleep(0.5)

        goal_handle.succeed()

        result = Dive.Result()
        result.final_depth = self.current_depth
        return result


def main(args=None):
    rclpy.init(args=args)
    dive_action_server = DiveActionServer()
    rclpy.spin(dive_action_server)


if __name__ == "__main__":
    main()
