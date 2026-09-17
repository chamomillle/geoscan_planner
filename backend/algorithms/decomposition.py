import numpy as np
from shapely.geometry import Polygon, LineString, box


def bcd_decomposition(polygon, no_fly_zones=None, step=10):
    """
    Boustrophedon Cellular Decomposition.
    Разбивает полигон на выпуклые ячейки.
    """
    if no_fly_zones:
        for nfz in no_fly_zones:
            polygon = polygon.difference(nfz)

    minx, miny, maxx, maxy = polygon.bounds
    cells = []

    prev_intervals = []
    current_cell_start = None

    for x in np.arange(minx, maxx + step, step):
        line = LineString([(x, miny - 10), (x, maxy + 10)])
        intersection = polygon.intersection(line)

        intervals = []
        if not intersection.is_empty:
            if intersection.geom_type == 'LineString':
                coords = list(intersection.coords)
                intervals = [(min(c[1] for c in coords), max(c[1] for c in coords))]
            elif intersection.geom_type == 'MultiLineString':
                for seg in intersection.geoms:
                    coords = list(seg.coords)
                    intervals.append((min(c[1] for c in coords), max(c[1] for c in coords)))

        if len(intervals) != len(prev_intervals):
            if current_cell_start is not None:
                cell = _extract_cell(polygon, current_cell_start, x - step)
                if cell and cell.area > 1:
                    cells.append(cell)
            current_cell_start = x

        prev_intervals = intervals

    if current_cell_start is not None:
        cell = _extract_cell(polygon, current_cell_start, maxx)
        if cell and cell.area > 1:
            cells.append(cell)

    return cells


def _extract_cell(polygon, x_start, x_end):
    """Извлекает кусок полигона между x_start и x_end."""
    minx, miny, maxx, maxy = polygon.bounds
    strip = box(x_start, miny - 10, x_end, maxy + 10)
    return polygon.intersection(strip)


def delaunay_decomposition(polygon, step=100):
    """
    Альтернатива: триангуляция Делоне.
    """
    from scipy.spatial import Delaunay
    from shapely.geometry import Point

    minx, miny, maxx, maxy = polygon.bounds
    points = []
    for x in np.arange(minx, maxx, step):
        for y in np.arange(miny, maxy, step):
            if polygon.contains(Point(x, y)):
                points.append((x, y))

    if len(points) < 3:
        return [polygon]

    points = np.array(points)
    tri = Delaunay(points)

    cells = []
    for simplex in tri.simplices:
        triangle_points = points[simplex]
        triangle = Polygon(triangle_points)
        if polygon.contains(triangle):
            cells.append(triangle)

    return cells