"""Load preplanned WayWise route files for the control tower route planner."""

from dataclasses import dataclass
import xml.etree.ElementTree as ET

from waywiser_teleop_py.osm_tiles import enu_to_llh, llh_to_enu


@dataclass
class RouteFilePoint:
    """A loaded route point transformed into the active map ENU frame."""

    x: float
    y: float
    z: float = 0.0
    speed: float = 1.0


def read_waywise_route_xml(filename, target_enuref):
    """Read a WayWise XML route and transform it into the target ENU frame."""
    tree = ET.parse(filename)
    root = tree.getroot()
    if root.tag != 'routes':
        raise ValueError('Expected a WayWise route XML file with <routes> as root.')

    imported_enuref = _read_enuref(root, target_enuref)
    points = []
    route_elem = root.find('route')
    if route_elem is None:
        return points

    for point_elem in route_elem.findall('point'):
        source_x = _read_float(point_elem, 'x', 0.0)
        source_y = _read_float(point_elem, 'y', 0.0)
        source_z = _read_float(point_elem, 'z', 0.0)
        speed = _read_float(point_elem, 'speed', 1.0)

        lat, lon, _ = enu_to_llh(source_x, source_y, imported_enuref)
        target_x, target_y = llh_to_enu(lat, lon, target_enuref)
        target_z = source_z + imported_enuref[2] - target_enuref[2]
        points.append(RouteFilePoint(target_x, target_y, target_z, speed))

    return points


def _read_enuref(root, fallback):
    enuref_elem = root.find('enuref')
    if enuref_elem is None:
        return [float(fallback[0]), float(fallback[1]), float(fallback[2])]
    return [
        _read_float(enuref_elem, 'Latitude', fallback[0]),
        _read_float(enuref_elem, 'Longitude', fallback[1]),
        _read_float(enuref_elem, 'Height', fallback[2]),
    ]


def _read_float(parent, tag, default):
    elem = parent.find(tag)
    if elem is None or elem.text is None:
        return float(default)
    return float(elem.text.strip())
