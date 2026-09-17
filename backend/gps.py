import math


def gps_to_local(lat, lon, center_lat, center_lon):
    meters_per_lat = 111320
    meters_per_lon = 111320 * math.cos(math.radians(center_lat))

    x = (lon - center_lon) * meters_per_lon
    y = (lat - center_lat) * meters_per_lat
    return x, y


def local_to_gps(x, y, center_lat, center_lon):
    meters_per_lat = 111320
    meters_per_lon = 111320 * math.cos(math.radians(center_lat))

    lat = center_lat + y / meters_per_lat
    lon = center_lon + x / meters_per_lon
    return [lat, lon]