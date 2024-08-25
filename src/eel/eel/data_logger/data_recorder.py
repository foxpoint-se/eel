# när det kommer ny position
#   kolla med latest_recorded
#       om den här är längre bort än 0.5 meter
#           bank()
#           spara lokation och djup
#
# om det kommer meddelande på /history/broadcast/cmd
#   publicera på /history/events
#
# när det finns connectivity
#   publicera regelbundet på /history/unpublished

# regelbundet, var 10e sekund
#

import math

from typing import List, Union
from .common import (
    Coord3d,
    LatLon,
    TimedCoord3d,
    Segment,
)


def get_3d_distance(coord1: Coord3d, coord2: Coord3d) -> float:
    delta_x = coord1["x"] - coord2["x"]
    delta_y = coord1["y"] - coord2["y"]
    delta_z = coord1["z"] - coord2["z"]
    return math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)


class PathRecorder:
    def __init__(
        self,
        meters_threshold: float,
        seconds_threshold: float,
    ) -> None:
        self.last_recorded_3d_position = None
        self.meters_threshold = meters_threshold
        self.seconds_threshold = seconds_threshold
        self.current_latlon: Union[LatLon, None] = None
        self.current_depth: Union[float, None] = None
        self.seconds_threshold = 2.0
        self.segment_in_progress: Union[Segment, None] = None
        self.last_finalized_segment: Union[Segment, None] = None
        self.finalized_segments: List[Segment] = []

    def _has_moved_significantly(self, next_to_record: Coord3d) -> bool:
        if self.last_recorded_3d_position is None:
            return True
        distance = get_3d_distance(next_to_record, self.last_recorded_3d_position)
        return distance >= self.meters_threshold

    def update_3d_position(self, new_pos: Coord3d) -> None:
        self.last_recorded_3d_position = new_pos

    def update_position(self, new_pos: LatLon) -> None:
        self.current_latlon = new_pos

    def update_depth(self, new_depth: float) -> None:
        self.current_depth = new_depth

    def update_connectivity(self, new_connectivity: bool) -> None:
        pass

    def init_segment(self, pos: TimedCoord3d) -> Segment:
        return Segment(
            accumulated_distance=0.0,
            ended_at_seconds=pos["created_at"],
            finalized=False,
            polyline=[pos],
            started_at_seconds=pos["created_at"],
        )

    def simplify_path(self, path: List[TimedCoord3d]) -> List[TimedCoord3d]:
        # return [c["coord"] for c in path]
        return [c for c in path]

    def get_path_distance(self, path: List[TimedCoord3d]) -> float:
        accumulated_distance = 0.0
        for index, coord in enumerate(path):
            if index != 0:
                prev = path[index - 1]
                dist = get_3d_distance(prev["coord"], coord["coord"])
                accumulated_distance += dist
        return accumulated_distance

    def finalize_segment(self, segment: Segment) -> Segment:
        start_time = segment["polyline"][0]["created_at"]
        end_time = segment["polyline"][-1]["created_at"]

        path = self.simplify_path(path=segment["polyline"])
        path_distance = self.get_path_distance(path)

        return Segment(
            started_at_seconds=start_time,
            ended_at_seconds=end_time,
            accumulated_distance=path_distance,
            finalized=True,
            polyline=path,
        )

    def add_pos_to_running(self, segment: Segment, pos: TimedCoord3d) -> None:
        total_distance = segment["accumulated_distance"] + get_3d_distance(
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

        else:
            finalized_segment = self.finalize_segment(self.segment_in_progress)
            if len(finalized_segment) > 0:
                last_from_previous = finalized_segment["polyline"][-1]
                self.segment_in_progress = self.init_segment(last_from_previous)
                self.add_pos_to_running(self.segment_in_progress, new_pos)
            else:
                self.segment_in_progress = self.init_segment(new_pos)

            if finalized_segment["accumulated_distance"] >= self.meters_threshold:
                self.last_finalized_segment = finalized_segment
                self.finalized_segments.append(finalized_segment)
            return

    def get_finalized_segments(self) -> List[Segment]:
        return self.finalized_segments

    def get_segment_in_progress(self) -> Union[Segment, None]:
        return self.segment_in_progress

    def set_thresholds(self, meters_threshold: float, seconds_threshold: float) -> None:
        self.meters_threshold = meters_threshold
        self.seconds_threshold = seconds_threshold
