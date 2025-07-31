import math
import json
import csv
from io import StringIO
from constants import FORBIDDEN_ZONES_DELTA_PATH

def haversine(lat1, lon1, lat2, lon2):
    """
    Вычисляет расстояние между двумя точками на сфере по формуле гаверсинуса.

    Args:
        lat1, lon1 (float): Координаты первой точки.
        lat2, lon2 (float): Координаты второй точки.

    Returns:
        float: Расстояние в метрах.
    """
    R = 6366037  # radius of Earth in meters
    phi_1 = math.radians(lat1)
    phi_2 = math.radians(lat2)

    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    meters = R * c
    meters = round(meters, 3)
    return meters


def cast_wrapper(element, cast_function):
    """
    Обёртка для безопасного приведения типов.
    
    Args:
        element: Элемент для приведения типа.
        cast_function: Функция приведения типа.

    Returns:
        Результат приведения типа или None в случае ошибки.
    """
    if element is None: 
        return None
    try:
        return cast_function(element)
    except ValueError:
        return None
    

def get_new_polygon_feature(name, coordinates):
    """
    Создаёт новый объект полигона для GeoJSON.

    Args:
        name (str): Имя полигона.
        coordinates (list): Список координат полигона.

    Returns:
        dict: Объект полигона в формате GeoJSON.
    """
    new_feature = {
        "type": "Feature",
        "properties": {
        "name": name
        },
        "geometry": {
        "type": "Polygon",
        "coordinates": [
            coordinates
        ]
        }
    }
    return new_feature


def is_point_in_polygon(point, polygon):
    """
    Проверяет, находится ли точка внутри полигона.

    Args:
        point (tuple): Координаты точки (x, y).
        polygon (list): Список координат вершин полигона.

    Returns:
        bool: True, если точка внутри полигона, иначе False.
    """
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def create_csv_from_telemetry(telemetry_data):
    """
    Создает CSV-строку из телеметрических данных.

    Args:
        telemetry_data (list): Список телеметрических данных.

    Returns:
        str: CSV-строка с телеметрическими данными.
    """
    output = StringIO()
    writer = csv.writer(output)
    writer.writerow(['record_time', 'lat', 'lon', 'alt', 'azimuth', 'dop', 'sats', 'speed'])

    for telemetry in telemetry_data:
        writer.writerow([telemetry.record_time, telemetry.lat, telemetry.lon, telemetry.alt, telemetry.azimuth, telemetry.dop, telemetry.sats, telemetry.speed])

    output.seek(0)
    return output.getvalue()


def compute_forbidden_zones_delta(old_zones, new_zones):
    """
    Вычисляет дельту изменений между старыми и новыми запрещенными зонами.

    Args:
        old_zones (dict): Старые запрещенные зоны.
        new_zones (dict): Новые запрещенные зоны.

    Returns:
        dict: Дельта изменений.
    """
    delta_zones = {"type": "FeatureCollection", "features": []}
    
    old_zones_dict = {zone['properties']['name']: zone for zone in old_zones['features']}
    new_zones_dict = {zone['properties']['name']: zone for zone in new_zones['features']}
    
    # Обработка добавленных и измененных зон
    for name, new_zone in new_zones_dict.items():
        if name not in old_zones_dict:
            new_zone['properties']['change_type'] = 'added'
            delta_zones['features'].append(new_zone)
        elif old_zones_dict[name]['geometry'] != new_zone['geometry']:
            new_zone['properties']['change_type'] = 'modified'
            delta_zones['features'].append(new_zone)
    
    # Обработка удаленных зон
    for name, old_zone in old_zones_dict.items():
        if name not in new_zones_dict:
            old_zone['properties']['change_type'] = 'deleted'
            delta_zones['features'].append(old_zone)
    
    return delta_zones


def compute_and_save_forbidden_zones_delta(old_zones, new_zones):
    try:
        delta_zones = compute_forbidden_zones_delta(old_zones, new_zones)
        
        with open(FORBIDDEN_ZONES_DELTA_PATH, 'w', encoding='utf-8') as f:
            json.dump(delta_zones, f, ensure_ascii=False, indent=4)
    except Exception as e:
        print(f"Error computing and saving forbidden zones delta: {e}")
        
        
def generate_forbidden_zones_string(forbidden_zones):
    """
    Генерирует строку запрещенных зон из JSON данных.

    Args:
        forbidden_zones (dict): JSON данные запрещенных зон.

    Returns:
        str: Строка запрещенных зон.
    """
    result_str = f'$ForbiddenZones {len(forbidden_zones["features"])}'
    for zone in forbidden_zones['features']:
        name = zone['properties']['name']
        coordinates = zone['geometry']['coordinates'][0]
        result_str += f'&{name}&{len(coordinates)}&{"&".join(list(map(lambda e: f"{e[1]:.7f}_{e[0]:.7f}", coordinates)))}'
    return result_str
