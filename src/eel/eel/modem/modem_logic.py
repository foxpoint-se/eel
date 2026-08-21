def modem_readings(reg_status: int | None, signal_strength: int | None) -> tuple[int, int] | None:
    if reg_status is None or signal_strength is None:
        return None
    return reg_status, signal_strength


def has_modem_readings(reg_status: int | None, signal_strength: int | None) -> bool:
    return modem_readings(reg_status, signal_strength) is not None


def modem_connectivity(
    reg_status: int,
    signal_strength: int,
    ping_ok: bool,
    *,
    min_signal_strength: int = 10,
) -> bool:
    registered = reg_status == 1 and signal_strength > min_signal_strength
    return ping_ok if registered else False
