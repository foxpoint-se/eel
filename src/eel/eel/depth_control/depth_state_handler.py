from enum import Enum
from .target_checker import TargetChecker
from .pid_controller import PidInterface
from .base import TankLevels, DepthConfig, EelDepthState
from .pid_merger import MergerInterface


class TargetState(Enum):
    NO_TARGET = 1
    TOWARDS_SURFACE = 2
    TOWARDS_TARGET = 3


class ProgressState(Enum):
    NO_ACTION = 1
    AT_SURFACE = 2
    AT_TARGET = 3


class DepthControllerInterface:
    def update(self, current_state: EelDepthState = None) -> TankLevels:
        """Get next state, given internal current state."""
        pass

    def set_new_target(self, depth_config: DepthConfig = None) -> None:
        """Set new target depth and pitch."""
        pass

    def get_target_state(self) -> TargetState:
        """Get current internal target state."""
        pass

    def get_progress_state(self) -> ProgressState:
        """Get current internal progress state."""
        pass


class DepthController(DepthControllerInterface):
    def __init__(
        self,
        target: DepthConfig = None,
        current_state: EelDepthState = None,
        depth_target_checker: TargetChecker = None,
        pitch_target_checker: TargetChecker = None,
        depth_velocity_target_checker: TargetChecker = None,
        pitch_velocity_target_checker: TargetChecker = None,
        depth_pid: PidInterface = None,
        pitch_pid: PidInterface = None,
        merger: MergerInterface = None,
        neutral_levels: TankLevels = None,
    ) -> None:
        super().__init__()
        self.__tank_levels: TankLevels = None
        self.__target = target
        self.__current_state = current_state
        self.__depth_target_checker = depth_target_checker
        self.__pitch_target_checker = pitch_target_checker
        self.__depth_velocity_target_checker = depth_velocity_target_checker
        self.__pitch_velocity_target_checker = pitch_velocity_target_checker
        self.__depth_pid = depth_pid
        self.__pitch_pid = pitch_pid
        self.__merger = merger
        self.__neutral_levels = neutral_levels

    def __are_tanks_empty(self):
        return (
            self.__current_state.front_tank_level < 0.05
            and self.__current_state.rear_tank_level < 0.05
        )

    def __get_levels_towards_surface(self) -> TankLevels:
        if self.get_progress_state() is ProgressState.AT_SURFACE:
            self.__target = None
            return None
        return TankLevels(front=0.0, rear=0.0)

    def __capture_neutral_levels(self):
        neutral_levels = TankLevels(
            front=self.__current_state.front_tank_level,
            rear=self.__current_state.rear_tank_level,
        )
        self.__neutral_levels = neutral_levels
        self.__merger.update_neutrals(neutral_levels)

    def update(self, current_state: EelDepthState = None) -> TankLevels:
        self.__current_state = current_state

        if self.__current_state is None:
            self.__tank_levels = None
            return self.__tank_levels

        if self.__target is None:
            self.__tank_levels = None
            return self.__tank_levels

        if self.get_target_state() is TargetState.TOWARDS_SURFACE:
            self.__tank_levels = self.__get_levels_towards_surface()
            return self.__tank_levels

        if self.get_target_state() is TargetState.TOWARDS_TARGET:
            if self.get_progress_state() is ProgressState.AT_TARGET:
                # if self.__are_all_at_target():
                self.__capture_neutral_levels()
                self.__tank_levels = self.__neutral_levels
                return self.__tank_levels
                # self.__tank_levels = TankLevels(self.__merger)
                # return

            # target = None |

            # if towards surface -> send surface levels
            # if towards target, send control signals
            #            also: update neutral levels.
            # if at target, send neutral

            # if towards target -> check all target checks.
            #   if they're not ok, send control signal
            #   if they're ok, save current levels as neutrals.
            #   send the neutrals.

            depth_control = self.__depth_pid.compute(
                self.__current_state.depth
            )
            pitch_control = self.__pitch_pid.compute(
                self.__current_state.pitch
            )

            merged = self.__merger.merge(
                depth_ctrl=TankLevels(front=depth_control, rear=depth_control),
                pitch_ctrl=TankLevels(
                    front=pitch_control, rear=-pitch_control
                ),
            )

            self.__tank_levels = merged
            return self.__tank_levels

        raise RuntimeError("this should not happen")

    def __are_all_at_target(self):
        return (
            self.__depth_target_checker.is_at_target(
                self.__current_state.depth
            )
            and self.__pitch_target_checker.is_at_target(
                self.__current_state.pitch
            )
            and self.__depth_velocity_target_checker.is_at_target(
                self.__current_state.depth_velocity
            )
            and self.__pitch_velocity_target_checker.is_at_target(
                self.__current_state.pitch_velocity
            )
        )

    def set_new_target(self, target: DepthConfig = None) -> None:
        self.__target = target

        updates = target
        if target is None:
            updates = DepthConfig(depth=None, pitch=None)
        self.__depth_pid.update_target(updates.depth)
        self.__pitch_pid.update_target(updates.pitch)
        self.__depth_target_checker.update_target(updates.depth)
        self.__pitch_target_checker.update_target(updates.pitch)

    def get_target_state(self) -> TargetState:
        if self.__target is None:
            return TargetState.NO_TARGET
        elif self.__target.depth < 0.05:
            return TargetState.TOWARDS_SURFACE
        return TargetState.TOWARDS_TARGET

    def get_progress_state(self) -> ProgressState:
        if self.__are_all_at_target():
            return ProgressState.AT_TARGET

        if self.__are_tanks_empty():
            return ProgressState.AT_SURFACE

        return ProgressState.NO_ACTION

    def get_neutral_levels(self) -> TankLevels:
        return self.__neutral_levels
