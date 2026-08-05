def command_is_stale(last_cmd_at: float | None, now: float, timeout_s: float) -> bool:
    """True when a command was seen before and silence exceeded timeout_s."""
    if last_cmd_at is None:
        return False
    return (now - last_cmd_at) > timeout_s


def require_positive_cmd_timeout(timeout_s: float) -> float:
    if timeout_s <= 0:
        raise ValueError(f"cmd_timeout_s must be > 0, got {timeout_s}")
    return timeout_s
