import math

EARTH_RADIUS = 6371000


def calc_distance(pos_1, pos_2):
    """Given two positions as longitude and latitude calculates the distance between the two positions in meters.

    :param pos_1: Position made up of longitude and latitude in signed decimal format
    :param pos_2: Position made up of longitude and latitude in signed decimal format
    :return Distance in meters between positions
    """
    pos_1_lat_rad = pos_1.get("latitude") * (math.pi / 180.0)
    pos_2_lat_rad = pos_2.get("latitude") * (math.pi / 180.0)
    delta_lat_rad = (pos_1.get("latitude") - pos_2.get("latitude")) * (math.pi / 180.0)
    delta_long_rad = (pos_1.get("longitude") - pos_2.get("longitude")) * (math.pi / 180.0)

    a = math.sin(delta_lat_rad / 2.0) * math.sin(delta_lat_rad / 2.0) + \
        math.cos(pos_1_lat_rad) * math.cos(pos_2_lat_rad) * math.sin(delta_long_rad / 2.0) * \
        math.sin(delta_long_rad / 2.0)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c

    return distance


def calc_bearing(pos_1, pos_2):
    """Given two positions as longitude and latitude calculates the bearing between the two positions in meters.

    :param pos_1: Position made up of longitude and latitude in signed decimal format
    :param pos_2: Position made up of longitude and latitude in signed decimal format
    :return: Bearing between the two points in true north degree format
    """
    pos_1_lat_rad = pos_1.get("latitude") * (math.pi / 180.0)
    pos_2_lat_rad = pos_2.get("latitude") * (math.pi / 180.0)
    delta_long_rad = (pos_1.get("longitude") - pos_2.get("longitude")) * (math.pi / 180.0)

    y = math.sin(delta_long_rad) * math.cos(pos_2_lat_rad)
    x = math.cos(pos_1_lat_rad) * math.sin(pos_2_lat_rad) - math.sin(pos_1_lat_rad) * math.cos(pos_2_lat_rad) * \
        math.cos(delta_long_rad)

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


if __name__ == "__main__":
    sample_pos_1 = {"latitude": 59.30766069495403, "longitude": 17.97525523434028}
    sample_pos_2 = {"latitude": 59.31200312176195, "longitude": 17.975909693329726}

    distance = calc_distance(sample_pos_1, sample_pos_2)
    bearing = calc_bearing(sample_pos_1, sample_pos_2)
    print(f"Calculated distance: {distance}\nCalculated bearing: {bearing}")
