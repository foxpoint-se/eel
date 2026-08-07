import json
from typing import cast

from ..utils.actuator_bounds import (
    is_finite_number,
    is_valid_coordinate,
    is_valid_depth_target,
    parse_depth_control_targets,
)
from ..utils.pid_tuning import parse_pid_type
from .payload_types import (
    AssignmentMqtt,
    BoolMsgMqtt,
    CoordinateMqtt,
    DepthControlCmdMqtt,
    FloatMsgMqtt,
    NavigationMissionMqtt,
    StringMqtt,
)


def _json_float(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if is_finite_number(number) else None
    if isinstance(value, str):
        try:
            number = float(value)
        except ValueError:
            return None
        return number if is_finite_number(number) else None
    return None


def parse_float_payload(payload: bytes) -> float | None:
    try:
        converted = cast(FloatMsgMqtt, json.loads(payload))
        return _json_float(converted["data"])
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None


def parse_bool_payload(payload: bytes) -> bool | None:
    try:
        converted = cast(BoolMsgMqtt, json.loads(payload))
        data = converted["data"]
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if not isinstance(data, bool):
        return None
    return data


def parse_string_payload(payload: bytes) -> str | None:
    try:
        converted = cast(StringMqtt, json.loads(payload))
        data = converted["data"]
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if not isinstance(data, str) or not data.strip():
        return None
    return data.strip()


def parse_coordinate_payload(payload: bytes) -> CoordinateMqtt | None:
    try:
        converted = cast(CoordinateMqtt, json.loads(payload))
        lat = _json_float(converted["lat"])
        lon = _json_float(converted["lon"])
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if lat is None or lon is None or not is_valid_coordinate(lat, lon):
        return None
    return {"lat": lat, "lon": lon}


def parse_depth_control_payload(payload: bytes) -> DepthControlCmdMqtt | None:
    try:
        converted = cast(DepthControlCmdMqtt, json.loads(payload))
        depth_target = _json_float(converted["depth_target"])
        pitch_target = _json_float(converted["pitch_target"])
        depth_pid_type = parse_pid_type(converted["depth_pid_type"])
        pitch_pid_type = parse_pid_type(converted["pitch_pid_type"])
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if depth_target is None or pitch_target is None:
        return None
    bounded = parse_depth_control_targets(depth_target, pitch_target)
    if bounded is None:
        return None
    depth_target, pitch_target = bounded
    if depth_pid_type is None or pitch_pid_type is None:
        return None
    return {
        "depth_target": depth_target,
        "pitch_target": pitch_target,
        "depth_pid_type": depth_pid_type,
        "pitch_pid_type": pitch_pid_type,
    }


def parse_mission_payload(payload: bytes) -> NavigationMissionMqtt | None:
    try:
        converted = cast(NavigationMissionMqtt, json.loads(payload))
        assignments = converted["assignments"]
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if not isinstance(assignments, list):
        return None

    parsed: list[AssignmentMqtt] = []
    for assignment in assignments:
        try:
            coord = assignment["coordinate"]
            lat = _json_float(coord["lat"])
            lon = _json_float(coord["lon"])
            target_depth = _json_float(assignment["target_depth"])
            sync_after = assignment["sync_after"]
        except (KeyError, TypeError, ValueError):
            return None
        if lat is None or lon is None or target_depth is None:
            return None
        if not isinstance(sync_after, bool):
            return None
        if not is_valid_coordinate(lat, lon) or not is_valid_depth_target(target_depth):
            return None
        parsed.append(
            {
                "coordinate": {"lat": lat, "lon": lon},
                "target_depth": target_depth,
                "sync_after": sync_after,
            }
        )
    return {"assignments": parsed}
