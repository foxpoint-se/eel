class TargetChecker:
    def is_at_target(self, value: float = None) -> bool:
        """Check if this state is within boundaries."""
        pass

    def update_target(self, value: float = None) -> None:
        """Update target."""
        pass


class BoundariesTargetChecker(TargetChecker):
    def __init__(
        self,
        upper_boundary: float = None,
        lower_boundary: float = None,
        target: float = None,
    ) -> None:
        super().__init__()
        self.__upper_boundary = upper_boundary
        self.__lower_boundary = lower_boundary
        self.__target = target

    def is_at_target(self, value: float = None) -> bool:
        if self.__target is None:
            return True

        if value is None:
            return True

        ceiling = self.__target + self.__upper_boundary
        floor = self.__target + self.__lower_boundary

        return value > floor and value < ceiling

    def update_target(self, target: float = None) -> None:
        self.__target = target


class VelocityTargetChecker(TargetChecker):
    def __init__(
        self, max_velocity: float = None, target: float = None
    ) -> None:
        super().__init__()
        self.__max_velocity = max_velocity
        self.__target = target

    def is_at_target(self, value: float = None) -> bool:
        if self.__target is None:
            return True

        if value is None:
            return True

        return abs(value) < self.__max_velocity

    def update_target(self, target: float = None) -> None:
        self.__target = target
