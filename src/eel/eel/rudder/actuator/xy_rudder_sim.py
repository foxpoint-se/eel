from .types import Vector2d
from .xy_rudder_base import XYRudder


class XYRudderSim(XYRudder):
    def set_rudder(self, value: Vector2d) -> None:
        pass

    def shutdown(self) -> None:
        pass
