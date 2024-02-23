def clamp(value: float, minimum: float, maximum: float):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value
