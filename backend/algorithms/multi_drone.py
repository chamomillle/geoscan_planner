import numpy as np
from shapely.geometry import Point, MultiPoint
from shapely.ops import unary_union


def split_polygon_kmeans(polygon, num_drones, cell_size=100):
    """
    Разбивает полигон на K подобластей через K-means.
    Возвращает список полигонов (по одному на дрон).
    """
    if num_drones <= 1:
        return [polygon]

    # Сетка точек внутри полигона
    minx, miny, maxx, maxy = polygon.bounds
    points = []
    x = minx + cell_size / 2
    while x < maxx:
        y = miny + cell_size / 2
        while y < maxy:
            if polygon.contains(Point(x, y)):
                points.append((x, y))
            y += cell_size
        x += cell_size

    if len(points) < num_drones * 3:
        return [polygon] * num_drones

    # K-means без sklearn (собственная реализация, чтобы не ставить лишние библиотеки)
    points_arr = np.array(points)

    # Инициализация: равномерно по индексам
    np.random.seed(42)
    indices = np.random.choice(len(points_arr), num_drones, replace=False)
    centroids = points_arr[indices].copy()

    for _ in range(50):
        # Присваиваем точки ближайшему центроиду
        distances = np.linalg.norm(
            points_arr[:, np.newaxis, :] - centroids[np.newaxis, :, :],
            axis=2
        )
        labels = np.argmin(distances, axis=1)

        # Пересчитываем центроиды
        new_centroids = []
        for i in range(num_drones):
            cluster_points = points_arr[labels == i]
            if len(cluster_points) > 0:
                new_centroids.append(cluster_points.mean(axis=0))
            else:
                new_centroids.append(centroids[i])

        new_centroids = np.array(new_centroids)

        if np.allclose(centroids, new_centroids):
            break
        centroids = new_centroids

    # Строим выпуклую оболочку для каждого кластера
    sub_polygons = []
    for i in range(num_drones):
        cluster_points = points_arr[labels == i]
        if len(cluster_points) < 3:
            continue

        hull = MultiPoint(cluster_points).convex_hull

        # Пересекаем с исходным полигоном (чтобы не выйти за границы)
        sub_poly = polygon.intersection(hull)

        if sub_poly.is_empty:
            continue

        # Если MultiPolygon — берём самый большой
        if sub_poly.geom_type == "MultiPolygon":
            sub_poly = max(sub_poly.geoms, key=lambda p: p.area)

        if sub_poly.geom_type == "Polygon" and sub_poly.area > 1:
            sub_polygons.append(sub_poly)

    # Если что-то не получилось — возвращаем исходный полигон
    if len(sub_polygons) < 2:
        return [polygon] * num_drones

    return sub_polygons


def lpt_distribute(cells, drones):
    """LPT-распределение ячеек между дронами."""
    cell_costs = [(i, cell.area) for i, cell in enumerate(cells)]
    cell_costs.sort(key=lambda x: -x[1])

    loads = {d['id']: 0 for d in drones}
    assignment = {d['id']: [] for d in drones}

    for cell_idx, cost in cell_costs:
        best_drone = min(drones, key=lambda d: loads[d['id']])
        assignment[best_drone['id']].append(cells[cell_idx])
        loads[best_drone['id']] += cost

    return assignment