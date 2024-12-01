from typing import Tuple, TypedDict


class CalibrationOffsets(TypedDict):
    mag: Tuple[int, int ,int]
    gyr: Tuple[int, int, int]
    acc: Tuple[int, int, int]
    