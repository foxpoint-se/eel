import json

from eel.mqtt_bridge.inbound_validation import (
    parse_bool_payload,
    parse_coordinate_payload,
    parse_depth_control_payload,
    parse_float_payload,
    parse_mission_payload,
    parse_string_payload,
)


def test__when_string_payload_has_surrounding_whitespace__should_return_stripped() -> None:
    assert parse_string_payload(json.dumps({"data": " rotholmen_runt_2025 "}).encode()) == "rotholmen_runt_2025"


def test__when_string_payload_is_whitespace_only__should_reject() -> None:
    assert parse_string_payload(json.dumps({"data": "   "}).encode()) is None


def test__when_bool_payload_is_string__should_reject() -> None:
    assert parse_bool_payload(json.dumps({"data": "false"}).encode()) is None


def test__when_float_payload_is_bool__should_reject() -> None:
    assert parse_float_payload(json.dumps({"data": True}).encode()) is None


def test__when_coordinate_payload_uses_bools__should_reject() -> None:
    assert parse_coordinate_payload(json.dumps({"lat": True, "lon": True}).encode()) is None


def test__when_depth_control_targets_are_bool__should_reject() -> None:
    payload = json.dumps(
        {
            "depth_target": True,
            "pitch_target": False,
            "depth_pid_type": "some_overshoot",
            "pitch_pid_type": "some_overshoot",
        }
    ).encode()
    assert parse_depth_control_payload(payload) is None


def test__when_depth_control_missing_pid_type__should_reject() -> None:
    payload = json.dumps({"depth_target": 1.0, "pitch_target": 0.0}).encode()
    assert parse_depth_control_payload(payload) is None


def test__when_mission_sync_after_is_string__should_reject() -> None:
    payload = json.dumps(
        {
            "assignments": [
                {
                    "coordinate": {"lat": 59.0, "lon": 18.0},
                    "target_depth": 1.0,
                    "sync_after": "false",
                }
            ]
        }
    ).encode()
    assert parse_mission_payload(payload) is None


def test__when_mission_coordinate_uses_bools__should_reject() -> None:
    payload = json.dumps(
        {
            "assignments": [
                {
                    "coordinate": {"lat": True, "lon": 18.0},
                    "target_depth": 1.0,
                    "sync_after": False,
                }
            ]
        }
    ).encode()
    assert parse_mission_payload(payload) is None
