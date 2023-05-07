from .depth_state_handler import DepthController
from .pid_controller import PidController
from .pid_merger import PidMerger
from .base import DepthConfig
from .target_checker import BoundariesTargetChecker, VelocityTargetChecker


depth_pid_1 = PidController(set_point=None, kP=0.25, kI=0.0, kD=0.0)
pitch_pid_1 = PidController(set_point=None, kP=0.02, kI=0.0, kD=0.127)

depth_target_checker_1 = BoundariesTargetChecker(
    upper_boundary=0.1, lower_boundary=-0.1, target=None
)
pitch_target_checker_1 = BoundariesTargetChecker(
    upper_boundary=7.0, lower_boundary=-7.0, target=None
)
depth_velocity_target_checker_1 = VelocityTargetChecker(max_velocity=0.1)
pitch_velocity_target_checker_1 = VelocityTargetChecker(max_velocity=3.0)
pid_merger_1 = PidMerger(
    front_boost=0.67,
    rear_boost=1.33,
    front_merge=DepthConfig(depth=0.8, pitch=0.2),
    rear_merge=DepthConfig(depth=0.8, pitch=0.2),
    front_neutral=0.44,
    rear_neutral=0.36,
)

pid_merger_for_simulation_1 = PidMerger(
    front_boost=0.67,
    rear_boost=1.33,
    front_merge=DepthConfig(depth=0.8, pitch=0.2),
    rear_merge=DepthConfig(depth=0.8, pitch=0.2),
    front_neutral=0.44,
    rear_neutral=0.36,
)

depth_controller_1 = DepthController(
    current_state=None,
    depth_pid=depth_pid_1,
    pitch_pid=pitch_pid_1,
    merger=pid_merger_1,
    depth_target_checker=depth_target_checker_1,
    pitch_target_checker=pitch_target_checker_1,
    depth_velocity_target_checker=depth_velocity_target_checker_1,
    pitch_velocity_target_checker=pitch_velocity_target_checker_1,
)

depth_controller_for_simulation_1 = DepthController(
    current_state=None,
    depth_pid=depth_pid_1,
    pitch_pid=pitch_pid_1,
    merger=pid_merger_for_simulation_1,
    depth_target_checker=depth_target_checker_1,
    pitch_target_checker=pitch_target_checker_1,
    depth_velocity_target_checker=depth_velocity_target_checker_1,
    pitch_velocity_target_checker=pitch_velocity_target_checker_1,
)
