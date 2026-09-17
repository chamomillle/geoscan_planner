from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import math
import tempfile

from shapely.geometry import Polygon, Point
from shapely.validation import make_valid

from gps import gps_to_local, local_to_gps
from algorithms.set_cover import (
    create_polygon_cells, create_polygon_candidates,
    select_polygon_points, build_route
)
from algorithms.lawnmower import best_lawnmower
from algorithms.decomposition import bcd_decomposition
from algorithms.multi_drone import split_polygon_kmeans
from algorithms.wind import correct_flight_time, optimal_lawnmower_angle
from algorithms.metrics import (
    route_length_m, flight_time_min,
    coverage_lawnmower, coverage_setcover, required_drones
)
from export.kml_export import export_route_to_kml, export_route_to_geojson

app = FastAPI(title="GEOSCAN Planner")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True,
    allow_methods=["*"], allow_headers=["*"],
)


class PlanRequest(BaseModel):
    coordinates: List[List[float]]
    no_fly_zones: List[List[List[float]]] = []
    landing_zones: List[List[float]] = []
    coverage_width: float = 150
    method: str = "set_cover"
    num_drones: int = 1
    wind_speed: float = 0
    wind_angle: float = 0
    altitude: float = 100
    fov_deg: float = 60
    drone_speed_kmh: float = 54
    drone_endurance_min: float = 40
    start_point: Optional[List[float]] = None


@app.get("/")
def index():
    return FileResponse(
        os.path.join(os.path.dirname(__file__), "..", "frontend", "index.html")
    )


def clean_polygon(polygon):
    """Приводит полигон к валидному виду."""
    if polygon.is_valid:
        return polygon

    fixed = make_valid(polygon)

    if fixed.geom_type == "MultiPolygon":
        fixed = max(fixed.geoms, key=lambda p: p.area)
    elif fixed.geom_type == "GeometryCollection":
        polys = [g for g in fixed.geoms if g.geom_type == "Polygon"]
        if not polys:
            return None
        fixed = max(polys, key=lambda p: p.area)
    elif fixed.geom_type != "Polygon":
        return None

    return fixed


def plan_one_drone(polygon, start_local, W, swath, method, wind_angle, wind_speed):
    """Планирует маршрут для одного дрона внутри полигона."""
    if method == "set_cover":
        cells = create_polygon_cells(polygon, 50)
        candidates = create_polygon_candidates(polygon, W / 2)
        selected = select_polygon_points(candidates, cells, W / 2)
        route = build_route(selected, start=start_local)
        return route, selected, cells

    else:  # lawnmower
        cells_shapely = bcd_decomposition(polygon)
        angle = optimal_lawnmower_angle(wind_angle) if wind_speed > 3 else 0

        all_waypoints = []
        for cell in cells_shapely:
            wp = best_lawnmower(cell, swath, angles=[angle, angle + 45, angle + 90])
            all_waypoints.extend(wp)

        if not all_waypoints:
            all_waypoints = [start_local]

        route = build_route(all_waypoints, start=start_local)
        return route, [], cells_shapely


@app.post("/api/plan")
async def plan(req: PlanRequest):
    if len(req.coordinates) < 3:
        raise HTTPException(400, "Нужно минимум 3 точки территории")

    clat = sum(p[0] for p in req.coordinates) / len(req.coordinates)
    clon = sum(p[1] for p in req.coordinates) / len(req.coordinates)

    local_coords = [gps_to_local(lat, lon, clat, clon) for lat, lon in req.coordinates]

    polygon = Polygon(local_coords)
    if not polygon.is_valid:
        polygon = clean_polygon(polygon)

    if polygon is None or polygon.is_empty or polygon.area < 1:
        raise HTTPException(400, "Полигон пустой или самопересекается. Нарисуйте заново.")

    # ==========================================
    # ВЫЧИТАЕМ ЗАПРЕТНЫЕ ЗОНЫ
    # ==========================================

    if req.no_fly_zones:
        for nfz_coords in req.no_fly_zones:
            if len(nfz_coords) < 3:
                continue
            nfz_local = [
                gps_to_local(lat, lon, clat, clon)
                for lat, lon in nfz_coords
            ]
            nfz_poly = Polygon(nfz_local)
            if not nfz_poly.is_valid:
                nfz_poly = clean_polygon(nfz_poly)
            if nfz_poly and not nfz_poly.is_empty and nfz_poly.area > 1:
                polygon = polygon.difference(nfz_poly)

    if polygon.is_empty or polygon.area < 1:
        raise HTTPException(400, "После вычитания запретных зон не осталось места для съёмки")

    # Если после difference получился MultiPolygon — берём самый большой
    if polygon.geom_type == "MultiPolygon":
        polygon = max(polygon.geoms, key=lambda p: p.area)

    W = req.coverage_width
    swath = W * 0.4

    if req.start_point:
        start_local = gps_to_local(req.start_point[0], req.start_point[1], clat, clon)
    else:
        start_local = local_coords[0]

    # ==========================================
    # ОДИН ДРОН
    # ==========================================

    if req.num_drones <= 1:
        route_local, selected, cells = plan_one_drone(
            polygon, start_local, W, swath, req.method, req.wind_angle, req.wind_speed
        )
        route_gps = [local_to_gps(x, y, clat, clon) for x, y in route_local]
        selected_gps = [local_to_gps(x, y, clat, clon) for x, y in selected]

        if req.method == "set_cover":
            coverage = coverage_setcover(selected, cells, W / 2)
        else:
            coverage = coverage_lawnmower(polygon, route_local, W)

        dist_m = route_length_m(route_local)
        time_min = flight_time_min(route_local, req.drone_speed_kmh)

        if req.wind_speed > 0:
            time_corrected = correct_flight_time(
                time_min, req.drone_speed_kmh, req.wind_speed, req.wind_angle
            ) or time_min
        else:
            time_corrected = time_min

        endurance_warning = time_corrected > req.drone_endurance_min
        min_drones = required_drones(time_corrected, req.drone_endurance_min)

        return {
            "method": req.method,
            "route": route_gps,
            "selected_points": selected_gps,
            "selected_points_count": len(selected_gps),
            "drone_routes": {},
            "no_fly_zones": req.no_fly_zones,
            "landing_zones": req.landing_zones,
            "metrics": {
                "route_length_km": round(dist_m / 1000, 2),
                "flight_time_min": round(time_corrected, 1),
                "flight_time_base_min": round(time_min, 1),
                "coverage_percent": round(coverage, 2),
                "num_drones": 1,
                "wind_applied": req.wind_speed > 0,
                "endurance_warning": endurance_warning,
                "recommended_drones": min_drones,
            },
            "territory": req.coordinates
        }

    # ==========================================
    # НЕСКОЛЬКО ДРОНОВ
    # ==========================================

    sub_polygons = split_polygon_kmeans(polygon, req.num_drones)

    drone_routes = {}
    drone_metrics = []
    total_distance = 0
    total_time = 0
    total_coverage = 0
    selected_gps_all = []

    for i, sub_poly in enumerate(sub_polygons):
        drone_id = f"drone_{i + 1}"

        route_local, selected, cells = plan_one_drone(
            sub_poly, start_local, W, swath, req.method, req.wind_angle, req.wind_speed
        )

        route_gps = [local_to_gps(x, y, clat, clon) for x, y in route_local]
        drone_routes[drone_id] = route_gps

        d_dist = route_length_m(route_local)
        d_time = flight_time_min(route_local, req.drone_speed_kmh)

        if req.wind_speed > 0:
            d_time = correct_flight_time(
                d_time, req.drone_speed_kmh, req.wind_speed, req.wind_angle
            ) or d_time

        total_distance += d_dist
        total_time += d_time
        drone_metrics.append({
            "drone_id": drone_id,
            "distance_km": round(d_dist / 1000, 2),
            "time_min": round(d_time, 1),
        })

        if req.method == "set_cover":
            sub_coverage = coverage_setcover(selected, cells, W / 2)
        else:
            sub_coverage = coverage_lawnmower(sub_poly, route_local, W)

        total_coverage += sub_coverage

        if selected:
            selected_gps_all.extend([
                local_to_gps(x, y, clat, clon) for x, y in selected
            ])

    avg_coverage = total_coverage / len(sub_polygons) if sub_polygons else 0
    makespan_val = max(m["time_min"] for m in drone_metrics) if drone_metrics else 0
    endurance_warning = makespan_val > req.drone_endurance_min

    return {
        "method": req.method,
        "route": drone_routes.get("drone_1", []),
        "selected_points": selected_gps_all,
        "selected_points_count": len(selected_gps_all),
        "drone_routes": drone_routes,
        "drone_metrics": drone_metrics,
        "no_fly_zones": req.no_fly_zones,
        "landing_zones": req.landing_zones,
        "metrics": {
            "route_length_km": round(total_distance / 1000, 2),
            "flight_time_min": round(makespan_val, 1),
            "flight_time_total_min": round(total_time, 1),
            "coverage_percent": round(avg_coverage, 2),
            "num_drones": len(sub_polygons),
            "wind_applied": req.wind_speed > 0,
            "endurance_warning": endurance_warning,
        },
        "territory": req.coordinates
    }


@app.post("/api/export/kml")
async def export_kml(data: dict):
    route = data.get("route")
    if not route or len(route) < 2:
        raise HTTPException(400, "Маршрут пустой")

    altitude = data.get("altitude", 100)
    fmt = data.get("format", "kml")

    with tempfile.NamedTemporaryFile(delete=False, suffix=f".{fmt}") as tmp:
        path = tmp.name

    if fmt == "kml":
        export_route_to_kml(route, path, altitude)
        media = "application/vnd.google-earth.kml+xml"
    else:
        export_route_to_geojson(route, path)
        media = "application/geo+json"

    return FileResponse(path, media_type=media, filename=f"route.{fmt}")


@app.post("/api/export/kml/multi")
async def export_kml_multi(data: dict):
    """Экспорт KML для нескольких дронов — разные цвета."""
    drone_routes = data.get("drone_routes", {})
    landing_zones = data.get("landing_zones", [])
    no_fly_zones = data.get("no_fly_zones", [])
    altitude = data.get("altitude", 100)

    if not drone_routes:
        raise HTTPException(400, "Нет маршрутов дронов")

    from simplekml import Kml
    kml = Kml()

    colors = ['ff0000ff', 'ff00ff00', 'ffff0000', 'ff00ffff',
              'ffff00ff', 'ff008080', 'ff808000', 'ff800000']

    for i, (drone_id, route) in enumerate(drone_routes.items()):
        if not route:
            continue
        linestring = kml.newlinestring(name=drone_id)
        linestring.coords = [(lon, lat, altitude) for lat, lon in route]
        linestring.altitudemode = 'absolute'
        linestring.style.linestyle.color = colors[i % len(colors)]
        linestring.style.linestyle.width = 3

    # Резервные площадки
    for j, (lat, lon) in enumerate(landing_zones):
        pnt = kml.newpoint(name=f"Landing_{j+1}", coords=[(lon, lat, 0)])
        pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/shapes/heliport.png"

    # Запретные зоны
    for k, nfz in enumerate(no_fly_zones):
        if len(nfz) < 3:
            continue
        pol = kml.newpolygon(name=f"NoFly_{k+1}")
        pol.outerboundaryis = [(lon, lat, 0) for lat, lon in nfz]
        pol.style.polystyle.color = '7f0000ff'
        pol.style.polystyle.fill = 1

    with tempfile.NamedTemporaryFile(delete=False, suffix=".kml") as tmp:
        path = tmp.name

    kml.save(path)

    return FileResponse(
        path,
        media_type="application/vnd.google-earth.kml+xml",
        filename="routes_multi.kml"
    )