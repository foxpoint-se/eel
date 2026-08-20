"""Derive tank mid-edge topics from the status topic prefix."""


def tank_level_topic(status_topic: str) -> str:
    return _tank_prefix(status_topic) + "/level"


def tank_pump_setpoint_topic(status_topic: str) -> str:
    return _tank_prefix(status_topic) + "/pump_setpoint"


def _tank_prefix(status_topic: str) -> str:
    suffix = "/status"
    if not status_topic.endswith(suffix):
        raise ValueError(f"status_topic must end with {suffix}, got {status_topic!r}")
    return status_topic[: -len(suffix)]
