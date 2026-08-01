from typing import TypeGuard, TypeVar

T = TypeVar("T")


def has_navigation_pose(current_position: T | None) -> TypeGuard[T]:
    return current_position is not None
