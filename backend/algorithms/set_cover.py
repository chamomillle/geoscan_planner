import math
from shapely.geometry import Point


def create_polygon_cells(polygon, cell_size):
    """
    Разбивает территорию на клетки.
    Берём только те, центр которых внутри полигона.
    """
    min_x, min_y, max_x, max_y = polygon.bounds
    cells = []

    x = min_x + cell_size / 2
    while x < max_x:
        y = min_y + cell_size / 2
        while y < max_y:
            if polygon.contains(Point(x, y)):
                cells.append((x, y))
            y += cell_size
        x += cell_size
    return cells


def create_polygon_candidates(polygon, step):
    """
    Создаём возможные точки съёмки внутри территории.
    """
    min_x, min_y, max_x, max_y = polygon.bounds
    candidates = []

    x = min_x + step / 2
    while x < max_x:
        y = min_y + step / 2
        while y < max_y:
            if polygon.contains(Point(x, y)):
                candidates.append((x, y))
            y += step
        x += step
    return candidates


def select_polygon_points(candidates, cells, coverage_radius):
    """
    Жадный Set Cover.
    На каждом шаге выбираем точку, покрывающую максимум непокрытых клеток.
    """
    uncovered = set(range(len(cells)))
    selected_points = []

    point_coverages = []
    for point in candidates:
        covered = set()
        for i, cell in enumerate(cells):
            dx = point[0] - cell[0]
            dy = point[1] - cell[1]
            if dx * dx + dy * dy <= coverage_radius * coverage_radius:
                covered.add(i)
        point_coverages.append(covered)

    while uncovered:
        best_index = None
        best_coverage = set()

        for i, coverage in enumerate(point_coverages):
            new_coverage = coverage & uncovered
            if len(new_coverage) > len(best_coverage):
                best_coverage = new_coverage
                best_index = i

        if best_index is None:
            break

        selected_points.append(candidates[best_index])
        uncovered -= best_coverage

    return selected_points


def calculate_polygon_coverage(point, cells, coverage_radius):
    """Какие клетки покрывает конкретная точка."""
    covered = set()
    camera = Point(point[0], point[1])

    for i, cell in enumerate(cells):
        if camera.distance(Point(cell[0], cell[1])) <= coverage_radius:
            covered.add(i)
    return covered


def build_route(points, start=(0, 0)):
    """Маршрут через ближайшего соседа."""
    if not points:
        return [start]

    route = [start]
    unvisited = list(points)
    current = start

    while unvisited:
        nearest = min(unvisited, key=lambda p: math.dist(current, p))
        route.append(nearest)
        unvisited.remove(nearest)
        current = nearest

    route.append(start)
    return route


def calculate_route_length(route):
    """Длина маршрута в метрах."""
    total = 0
    for i in range(len(route) - 1):
        total += math.dist(route[i], route[i + 1])
    return total