import math
from shapely.geometry import LineString, Point


def route_length_m(waypoints):
    """Длина маршрута в метрах."""
    if len(waypoints) < 2:
        return 0
    return sum(
        math.dist(waypoints[i], waypoints[i + 1])
        for i in range(len(waypoints) - 1)
    )


def flight_time_min(waypoints, speed_kmh, turn_radius_m=10):
    """Время полёта в минутах с учётом разворотов."""
    length = route_length_m(waypoints)
    speed_ms = speed_kmh / 3.6
    num_turns = max(0, len(waypoints) // 2 - 1)
    turn_time = num_turns * (math.pi * turn_radius_m / speed_ms)
    return (length / speed_ms + turn_time) / 60


def coverage_lawnmower(polygon, waypoints, swath_width):
    """Покрытие для змейки — через буфер вокруг линии маршрута."""
    if len(waypoints) < 2:
        return 0.0
    try:
        line = LineString(waypoints)
        buffered = line.buffer(swath_width / 2)
        intersection = polygon.intersection(buffered)
        if intersection.is_empty:
            return 0.0
        return intersection.area / polygon.area * 100
    except Exception:
        return 0.0


def coverage_setcover(selected_points, cells, coverage_radius):
    """
    Покрытие для Set Cover — через объединение кругов вокруг точек.
    """
    if not cells:
        return 0.0
    covered = set()
    for point in selected_points:
        for i, cell in enumerate(cells):
            dx = point[0] - cell[0]
            dy = point[1] - cell[1]
            if dx * dx + dy * dy <= coverage_radius * coverage_radius:
                covered.add(i)
    return len(covered) / len(cells) * 100


def coverage_percent(polygon, waypoints, swath_width):
    """Устаревшая функция — оставлена для совместимости."""
    return coverage_lawnmower(polygon, waypoints, swath_width)


def makespan(flight_times):
    """Максимальное время среди всех дронов."""
    return max(flight_times) if flight_times else 0


def total_flight_time(flight_times):
    """Суммарный налёт."""
    return sum(flight_times)


def required_drones(total_time_min, endurance_min):
    """Минимальное число дронов, чтобы уложиться в endurance."""
    if endurance_min <= 0:
        return 1
    return max(1, math.ceil(total_time_min / endurance_min))