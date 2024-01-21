percent_voltage_map = {
    100: 16.8,
    95: 16.6,
    90: 16.45,
    85: 16.33,
    80: 16.09,
    75: 15.93,
    70: 15.81,
    65: 15.66,
    60: 15.5,
    55: 15.42,
    50: 15.34,
    45: 15.26,
    40: 15.18,
    35: 15.14,
    30: 15.06,
    25: 14.99,
    20: 14.91,
    15: 14.83,
    10: 14.75,
    5: 14.43,
    0: 13.09,
}


def calculate_voltage_percent(voltage: float) -> float:
    percent = 0.0

    if voltage >= percent_voltage_map[100]:
        percent = 100.0
        return percent

    for mapped_percent, mapped_voltage in percent_voltage_map.items():
        if voltage < mapped_voltage:
            continue
        else:
            percent = mapped_percent
            break

    return percent
