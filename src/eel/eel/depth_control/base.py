class TankLevels:
    def __init__(self, front: float = None, rear: float = None) -> None:
        self.front = front
        self.rear = rear


class DepthConfig:
    def __init__(self, depth: float = None, pitch: float = None) -> None:
        self.depth = depth
        self.pitch = pitch


class EelDepthState:
    def __init__(
        self,
        depth: float = None,
        pitch: float = None,
        depth_velocity: float = None,
        pitch_velocity: float = None,
        front_tank_level: float = None,
        rear_tank_level: float = None,
    ) -> None:
        self.depth = depth
        self.pitch = pitch
        self.depth_velocity = depth_velocity
        self.pitch_velocity = pitch_velocity
        self.front_tank_level = front_tank_level
        self.rear_tank_level = rear_tank_level
