from .base import TankLevels, DepthConfig


class MergerInterface:
    def merge(
        self, depth_ctrl: TankLevels, pitch_ctrl: TankLevels
    ) -> TankLevels:
        """Merge two control signals to one."""
        pass


class PidMerger(MergerInterface):
    def __init__(
        self,
        front_boost: float = 1.0,
        rear_boost: float = 1.0,
        front_merge: DepthConfig = DepthConfig(depth=0.5, pitch=0.5),
        rear_merge: DepthConfig = DepthConfig(depth=0.5, pitch=0.5),
        front_neutral: float = 0.5,
        rear_neutral: float = 0.5,
    ) -> None:
        super().__init__()
        self.front_boost = front_boost
        self.rear_boost = rear_boost
        self.front_merge = front_merge
        self.rear_merge = rear_merge
        self.front_neutral = front_neutral
        self.rear_neutral = rear_neutral

    # pass in same for the first two. flip rear to be negative
    def merge(
        self,
        depth_ctrl: TankLevels = None,
        pitch_ctrl: TankLevels = None,
    ) -> TankLevels:
        if depth_ctrl is None and pitch_ctrl is None:
            return TankLevels()
        combined_front = (self.front_merge.depth * depth_ctrl.front) + (
            self.front_merge.pitch * pitch_ctrl.front
        )

        boosted_front = self.front_boost * combined_front

        combined_rear = (self.rear_merge.depth * depth_ctrl.rear) + (
            self.rear_merge.pitch * pitch_ctrl.rear
        )

        boosted_rear = self.rear_boost * combined_rear

        front_with_offset = boosted_front + self.front_neutral
        rear_with_offset = boosted_rear + self.rear_neutral

        return TankLevels(front=front_with_offset, rear=rear_with_offset)

    def update_neutral(self, front_neutral, rear_neutral):
        self.front_neutral = front_neutral
        self.rear_neutral = rear_neutral
