from .depth_state_handler import DepthController
from .pid_controller import PidController
from .pid_merger import PidMerger
from .base import DepthConfig, TankLevels
from .target_checker import BoundariesTargetChecker, VelocityTargetChecker


sim_depth_pid_1 = PidController(set_point=None, kP=0.5, kI=0.0, kD=1.0)
depth_pid_1 = PidController(set_point=None, kP=0.25, kI=0.0, kD=0.0)
pitch_pid_1 = PidController(set_point=None, kP=0.02, kI=0.0, kD=0.127)
sim_pitch_pid_1 = PidController(set_point=None, kP=0.1, kI=0.0, kD=0.5)

depth_target_checker_1 = BoundariesTargetChecker(
    upper_boundary=0.1, lower_boundary=-0.1, target=None
)
pitch_target_checker_1 = BoundariesTargetChecker(
    upper_boundary=7.0, lower_boundary=-7.0, target=None
)
sim_pitch_target_checker_1 = BoundariesTargetChecker(
    upper_boundary=2.0, lower_boundary=-2.0, target=None
)

sim_neutrals = TankLevels(0.5, 0.5)
prod_neutrals = TankLevels(front=0.18, rear=0.69)

depth_velocity_target_checker_1 = VelocityTargetChecker(max_velocity=0.05)
pitch_velocity_target_checker_1 = VelocityTargetChecker(max_velocity=3.0)
pid_merger_1 = PidMerger(
    front_boost=0.67,
    rear_boost=1.33,
    front_merge=DepthConfig(depth=0.8, pitch=0.2),
    rear_merge=DepthConfig(depth=0.8, pitch=0.2),
    front_neutral=prod_neutrals.front,
    rear_neutral=prod_neutrals.rear,
)

pid_merger_for_simulation_1 = PidMerger(
    front_boost=1.0,
    rear_boost=1.0,
    front_merge=DepthConfig(depth=0.5, pitch=0.5),
    rear_merge=DepthConfig(depth=0.5, pitch=0.5),
    front_neutral=sim_neutrals.front,
    rear_neutral=sim_neutrals.rear,
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
    neutral_levels=prod_neutrals,
)

depth_controller_for_simulation_1 = DepthController(
    current_state=None,
    depth_pid=sim_depth_pid_1,
    pitch_pid=sim_pitch_pid_1,
    merger=pid_merger_for_simulation_1,
    depth_target_checker=depth_target_checker_1,
    pitch_target_checker=sim_pitch_target_checker_1,
    depth_velocity_target_checker=depth_velocity_target_checker_1,
    pitch_velocity_target_checker=pitch_velocity_target_checker_1,
    neutral_levels=sim_neutrals,
)
