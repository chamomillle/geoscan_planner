import json
from simplekml import Kml


def export_route_to_kml(route_gps, filename="route.kml", altitude=100):
    """
    Экспорт маршрута в KML.
    route_gps: [[lat, lon], ...]
    """
    kml = Kml()
    linestring = kml.newlinestring(name="Маршрут БВС")
    linestring.coords = [(lon, lat, altitude) for lat, lon in route_gps]
    linestring.altitudemode = 'absolute'
    linestring.extrude = 1

    for i, (lat, lon) in enumerate(route_gps):
        kml.newpoint(name=f"WP{i + 1}", coords=[(lon, lat, altitude)])

    kml.save(filename)
    return filename


def export_route_to_geojson(route_gps, filename="route.geojson"):
    """Экспорт маршрута в GeoJSON."""
    geojson = {
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": [[lon, lat] for lat, lon in route_gps]
        },
        "properties": {"name": "Маршрут БВС"}
    }
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(geojson, f, ensure_ascii=False, indent=2)
    return filename