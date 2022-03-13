import math

EARTH_RADIUS = 6371000


def get_distance_in_meters(lat_1, lon_1, lat_2, lon_2):
    """Given two positions as longitude and latitude calculates the distance between the two positions in meters.

    :param lat_1: Latitude for first position in signed decimal format
    :param lon_1: Longitude for first position in signed decimal format
    :param lat_2: Latitude for second position in signed decimal format
    :param lon_2: Longitude for second position in signed decimal format
    :return Distance in meters between positions
    """
    lat_1_rad = lat_1 * (math.pi / 180.0)
    lat_2_rad = lat_2 * (math.pi / 180.0)
    delta_lat_rad = (lat_1 - lat_2) * (math.pi / 180.0)
    delta_long_rad = (lon_1 - lon_2) * (math.pi / 180.0)

    a = math.sin(delta_lat_rad / 2.0) * math.sin(delta_lat_rad / 2.0) + math.cos(
        lat_1_rad
    ) * math.cos(lat_2_rad) * math.sin(delta_long_rad / 2.0) * math.sin(
        delta_long_rad / 2.0
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c

    return distance


def get_relative_bearing_in_degrees(lat_1, lon_1, lat_2, lon_2):
    """Given two positions as longitude and latitude calculates the bearing between the two positions in meters.

    :param lat_1: Latitude for first position in signed decimal format
    :param lon_1: Longitude for first position in signed decimal format
    :param lat_2: Latitude for second position in signed decimal format
    :param lon_2: Longitude for second position in signed decimal format
    :return: Relative bearing between the two points in true north degree format
    """
    lat_1_rad = lat_1 * (math.pi / 180.0)
    lat_2_rad = lat_2 * (math.pi / 180.0)
    delta_long_rad = (lon_1 - lon_2) * (math.pi / 180.0)

    y = math.sin(delta_long_rad) * math.cos(lat_2_rad)
    x = math.cos(lat_1_rad) * math.sin(lat_2_rad) - math.sin(lat_1_rad) * math.cos(
        lat_2_rad
    ) * math.cos(delta_long_rad)

    bearing = math.atan2(y, x) * (180.0 / math.pi)
    true_bearing = bearing * -1 if bearing < 0 else 360.0 - bearing

    return true_bearing


def convert_deg_min_to_deg(position):
    """
    Converts a position either long or lat from the deg min format into decimal degrees.
    Default NMEA format is deg min format.

    :param position: Either longitude or latitude in deg minute format
    :return: Position in degree decimal format
    """
    deg = int(position)
    decimal = position - deg

    return deg + (decimal / 60)


def get_closest_turn_direction(current_heading, target_heading):
    """
    Given current heading and target heading, get closest direction to turn, right or left.

    :param current_heading: In degrees
    :param target_heading: In degrees
    :return: -1 for left and 1 for right
    """
    diff = target_heading - current_heading

    if diff < 0:
        diff += 360

    return -1 if diff > 180 else 1


if __name__ == "__main__":
    sample_pos_1 = {"latitude": 59.30766069495403, "longitude": 17.97525523434028}
    sample_pos_2 = {"latitude": 59.31200312176195, "longitude": 17.975909693329726}

    distance = get_distance_in_meters(
        sample_pos_1["latitude"],
        sample_pos_1["longitude"],
        sample_pos_2["latitude"],
        sample_pos_2["longitude"],
    )
    bearing = get_relative_bearing_in_degrees(
        sample_pos_1["latitude"],
        sample_pos_1["longitude"],
        sample_pos_2["latitude"],
        sample_pos_2["longitude"],
    )
    print(f"Calculated distance: {distance}\nCalculated bearing: {bearing}")
