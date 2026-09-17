import math
from shapely.geometry import LineString, Point
from shapely.affinity import rotate


def swath_width_from_fov(height_m, fov_deg):
    """Ширина захвата камеры."""
    return 2 * height_m * math.tan(math.radians(fov_deg / 2))


def lawnmower_in_cell(cell, swath_width, angle_deg=0):
    """Змейка внутри одной выпуклой ячейки."""
    rotated = rotate(cell, -angle_deg, origin='centroid')
    minx, miny, maxx, maxy = rotated.bounds

    waypoints = []
    y = miny + swath_width / 2
    direction = 1

    while y <= maxy:
        line = LineString([(minx - 10, y), (maxx + 10, y)])
        intersection = rotated.intersection(line)

        if not intersection.is_empty:
            if intersection.geom_type == 'LineString':
                coords = list(intersection.coords)
                waypoints.extend(coords if direction == 1 else reversed(coords))
            elif intersection.geom_type == 'MultiLineString':
                for seg in intersection.geoms:
                    coords = list(seg.coords)
                    waypoints.extend(coords if direction == 1 else reversed(coords))

        y += swath_width
        direction *= -1

    if angle_deg != 0:
        waypoints = [
            rotate(Point(p), angle_deg, origin=cell.centroid).coords[0]
            for p in waypoints
        ]

    return waypoints


def best_lawnmower(cell, swath_width, angles=None):
    """Перебирает ориентации, выбирает самую короткую."""
    if angles is None:
        angles = [0, 45, 90, 135]

    best_wp = None
    best_len = float('inf')

    for a in angles:
        wp = lawnmower_in_cell(cell, swath_width, a)
        if len(wp) < 2:
            continue
        length = sum(math.dist(wp[i], wp[i + 1]) for i in range(len(wp) - 1))
        if length < best_len:
            best_len = length
            best_wp = wp

    return best_wp or []


def generate_lawnmower(polygon, swath_width, angle_deg=0):
    """Змейка по всему полигону (для простых выпуклых областей)."""
    return lawnmower_in_cell(polygon, swath_width, angle_deg)