import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

from ..utils.pid_controller import PidController

from eel_interfaces.action import Dive
from eel_interfaces.msg import ImuStatus, PressureStatus

from ..utils.topics import (
    PRESSURE_STATUS,
    RUDDER_Y_CMD,
    IMU_STATUS
)


UPDATE_FREQUENCY_PER_SEC = 5


class DiveActionServer(Node):
    def __init__(self):
        super().__init__("dive_action_server")
        self._action_server = ActionServer(
            self,
            Dive,
            "dive",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_msg, 10)
        self.create_subscription(PressureStatus, PRESSURE_STATUS, self.handle_pressure_msg, 10)        
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_Y_CMD, 10)

        self.depth_target = 0.0
        self.max_dive_angle = 30.0
        self.max_rudder_output = 1.0
        self.max_dive_depth_m = 5.0

        self.dive_time = 0.0
        self.max_dive_time_sec = 120.0

        self.angle_pid_output = 0.0
        self.angle_pid_controller = PidController(0.0, kP=-35, kD=-5) # Inner PID
        self.rudder_pid_output = 0.0
        self.rudder_pid_controller = PidController(0.0, kP=1 / 15)

        self.current_depth = 0.0
        self.current_pitch = 0.0
        self.valid_depth_offset = 0.15

        self.new_sensor_data = False

        self.logger = self.get_logger()
        self.logger.info("Dive action server started")

    def handle_imu_msg(self, msg):
        self.current_pitch = msg.pitch
        self.new_sensor_data = True
    
    def handle_pressure_msg(self, msg):
        self.current_depth = msg.depth
    
    def compute_and_send_rudder_value(self):
        self.angle_pid_output = self.compute_new_target_angle()
        self.rudder_pid_output = self.compute_new_rudder_output(self.angle_pid_output)

        self.send_rudder_msg(self.rudder_pid_output)

    def send_rudder_msg(self, rudder_value):
        rudder_msg = Float32()
        rudder_msg.data = float(rudder_value)
        self.rudder_publisher.publish(rudder_msg)

    def compute_new_target_angle(self):
        pid_angle_output = self.angle_pid_controller.compute(self.current_depth)

        # Check if the PID is requesting angles larger then max value
        if abs(pid_angle_output) > self.max_dive_angle:
            if pid_angle_output > 0:
                pid_angle_output = self.max_dive_angle
            else:
                pid_angle_output = -1 * self.max_dive_angle
        
        return -1 * pid_angle_output
    
    def compute_new_rudder_output(self, target_angle):
        self.rudder_pid_controller.update_set_point(target_angle)
        rudder_output = self.rudder_pid_controller.compute(self.current_pitch)

        # Check if the PID is requesting servo values larger then max value
        if abs(rudder_output) > self.max_rudder_output:
            if rudder_output > 0:
                rudder_output = self.max_rudder_output
            else:
                rudder_output = -1 * self.max_rudder_output
        
        return rudder_output

    def goal_callback(self, goal_request):
        # Sanity check for depth and dive that the values are not to large
        depth_target_larger_then_max_depth = goal_request.wanted_depth > self.max_dive_depth_m
        dive_time_larger_then_max_dive_time = self.dive_time > self.max_dive_time_sec
        
        if any([depth_target_larger_then_max_depth, dive_time_larger_then_max_dive_time]):
            self.logger.info(f"Invalid request parameters, will reject goal.")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.depth_target = goal_handle.request.wanted_depth
        self.dive_time = goal_handle.request.dive_time
        self.angle_pid_controller.update_set_point(self.depth_target)
        start_ts = time.time()
        depth_reached_after = 0.0

        self.logger.info(f"Executing goal, will dive to {self.depth_target}m for {self.dive_time}s")

        feedback_msg = Dive.Feedback()
        result = Dive.Result()

        # Action server will do two things, first dive to a depth for X sec
        while time.time() - start_ts < self.dive_time:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.logger.info(f"Goal cancelled will start to ascend")
                break

            if depth_reached_after == 0.0 and self.current_depth >= self.depth_target:
                depth_reached_after = time.time() - start_ts

            # What is triggering PID-recalculations is new IMU readings
            if self.new_sensor_data:
                self.compute_and_send_rudder_value()
                self.new_sensor_data = False

                feedback_msg.current_depth = self.current_depth
                feedback_msg.time_left = self.dive_time - (time.time() - start_ts)
                feedback_msg.depth_reached_after = depth_reached_after
                feedback_msg.pid_angle_output = self.angle_pid_output
                feedback_msg.pid_rudder_output = self.rudder_pid_output
                goal_handle.publish_feedback(feedback_msg)

        self.depth_target = 0.0
        self.angle_pid_controller.update_set_point(self.depth_target)
        min_offset = self.depth_target - self.valid_depth_offset
        max_offset = self.depth_target + self.valid_depth_offset

        # Secondly once time has passed, go back to surface
        while not (min(min_offset, max_offset) < self.current_depth < max(min_offset, max_offset)):
            # What is triggering PID-recalculations is new IMU readings
            if self.new_sensor_data:
                self.compute_and_send_rudder_value()
                self.new_sensor_data = False

                feedback_msg.current_depth = self.current_depth
                feedback_msg.time_left = self.dive_time - (time.time() - start_ts)
                feedback_msg.pid_angle_output = self.angle_pid_output
                feedback_msg.pid_rudder_output = self.rudder_pid_output
                goal_handle.publish_feedback(feedback_msg)

        self.send_rudder_msg(0.0)
        goal_handle.succeed()
        
        result.final_depth = self.current_depth
        result.action_time = time.time() - start_ts

        return result


def main(args=None):
    rclpy.init(args=args)
    dive_action_server = DiveActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(dive_action_server, executor=executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
