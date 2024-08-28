from typing import Callable, List, Union
from .common import (
    Coord3d,
    TimedCoord3d,
    Segment,
)
from ..utils.nav import (
    get_distance_in_meters,
)


# TODO:
# - change topic names


def get_2d_distance_earth(coord1: Coord3d, coord2: Coord3d) -> float:
    return get_distance_in_meters(
        coord1["lat"],
        coord1["lon"],
        coord2["lat"],
        coord2["lon"],
    )


class PathRecorder:
    def __init__(
        self,
        meters_threshold: float,
        seconds_threshold: float,
        on_new_segment: Callable[[Segment], None],
    ) -> None:
        self.last_recorded_3d_position = None
        self.meters_threshold = meters_threshold
        self.seconds_threshold = seconds_threshold
        self.segment_in_progress: Union[Segment, None] = None
        self.finalized_segments: List[Segment] = []
        self.on_new_segment = on_new_segment

    def init_segment(self, pos: TimedCoord3d) -> Segment:
        return Segment(
            accumulated_distance=0.0,
            ended_at_seconds=pos["created_at"],
            finalized=False,
            polyline=[pos],
            started_at_seconds=pos["created_at"],
        )

    def get_path_distance(self, path: List[TimedCoord3d]) -> float:
        accumulated_distance = 0.0
        for index, coord in enumerate(path):
            if index != 0:
                prev = path[index - 1]
                dist = get_2d_distance_earth(prev["coord"], coord["coord"])
                accumulated_distance += dist
        return accumulated_distance

    def finalize_segment(self, segment: Segment) -> Segment:
        first = segment["polyline"][0]
        last = segment["polyline"][-1]
        start_time = first["created_at"]
        end_time = last["created_at"]

        # TODO: we could simplify the path here.
        # but that would involve having to ensure that the two endpoints are correct
        # as well as setting the depth for each new coordinate, based on some sort of
        # algorithm to determine it from the adjacent coordinates in the original path
        # or something
        path_distance = self.get_path_distance(segment["polyline"])

        simplified_polyline = [first, last]

        return Segment(
            started_at_seconds=start_time,
            ended_at_seconds=end_time,
            accumulated_distance=path_distance,
            finalized=True,
            polyline=simplified_polyline,
        )

    def add_pos_to_running(self, segment: Segment, pos: TimedCoord3d) -> None:
        total_distance = segment["accumulated_distance"] + get_2d_distance_earth(
            segment["polyline"][-1]["coord"], pos["coord"]
        )
        segment["polyline"].append(pos)
        segment["accumulated_distance"] = total_distance

    def step(self, new_pos: TimedCoord3d) -> None:
        if not self.segment_in_progress:
            self.segment_in_progress = self.init_segment(new_pos)
            return

        segment_start = self.segment_in_progress["polyline"][0]["created_at"]

        if new_pos["created_at"] - segment_start < self.seconds_threshold:
            self.add_pos_to_running(self.segment_in_progress, new_pos)
            return

        finalized_segment = self.finalize_segment(self.segment_in_progress)
        if len(finalized_segment["polyline"]) > 0:
            last_from_previous = finalized_segment["polyline"][-1]
            self.segment_in_progress = self.init_segment(last_from_previous)
            self.add_pos_to_running(self.segment_in_progress, new_pos)
        else:
            self.segment_in_progress = self.init_segment(new_pos)

        if finalized_segment["accumulated_distance"] >= self.meters_threshold:
            self.finalized_segments.append(finalized_segment)
            self.on_new_segment(finalized_segment)

    def get_finalized_segments(self) -> List[Segment]:
        return self.finalized_segments

    def get_segment_in_progress(self) -> Union[Segment, None]:
        return self.segment_in_progress

    def set_thresholds(self, meters_threshold: float, seconds_threshold: float) -> None:
        self.meters_threshold = meters_threshold
        self.seconds_threshold = seconds_threshold
