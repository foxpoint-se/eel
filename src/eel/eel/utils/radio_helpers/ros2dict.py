import numpy as np
import array


def ros2dict(msg):
    if type(msg) in (str, bool, int, float):
        return msg

    output = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field)
        if type(value) in (str, bool, int, float):
            output[field] = value

        elif type(value) is list:
            output[field] = [ros2dict(el) for el in value]

        elif type(value) in (np.ndarray, array.array):
            output[field] = [ros2dict(el) for el in value.tolist()]

        else:
            output[field] = ros2dict(value)

    return output
