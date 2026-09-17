import math


def ground_speed(drone_speed_kmh, wind_speed_ms, wind_angle_deg):
    """
    Путевая скорость с учётом ветра.
    wind_angle_deg: 0 = попутный, 180 = встречный.
    """
    drone_ms = drone_speed_kmh / 3.6
    wind_component = wind_speed_ms * math.cos(math.radians(wind_angle_deg))
    effective = drone_ms + wind_component
    return max(effective, 1.0) * 3.6


def correct_flight_time(base_time_min, drone_speed_kmh, wind_speed_ms, wind_angle_deg):
    """Скорректированное время полёта с учётом ветра."""
    drone_ms = drone_speed_kmh / 3.6
    wind_component = wind_speed_ms * math.cos(math.radians(wind_angle_deg))
    effective_ms = drone_ms + wind_component

    if effective_ms <= 0:
        return None

    return base_time_min * (drone_ms / effective_ms)


def optimal_lawnmower_angle(wind_angle_deg):
    """Оптимальная ориентация змейки: развороты против ветра."""
    return (wind_angle_deg + 90) % 180