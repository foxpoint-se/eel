"""Depth → fake modem radio fields (same rules as old ModemSimulator / modem_sim)."""


def simulated_signal_strength(depth_m: float) -> int:
    if depth_m < 0.1:
        return 31
    if 0.1 < depth_m < 0.2:
        return 18
    return 0


def simulated_registration_status(depth_m: float) -> int:
    return 1 if depth_m < 0.2 else 0
