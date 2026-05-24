"""Interactive route planning map widgets for the control tower."""

from dataclasses import dataclass
import math

from PyQt5.QtCore import pyqtSignal, QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QCursor, QFont, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import QDoubleSpinBox, QHBoxLayout, QLabel, QMenu, QWidget, QWidgetAction

from waywiser_teleop_py.osm_tiles import (
    draw_tile_placeholder,
    enu_to_llh,
    lat_to_tile_y,
    llh_to_enu,
    lon_to_tile_x,
    OsmTileClient,
    TILE_SIZE_PX,
    tile_x_to_lon,
    tile_y_to_lat,
)


OSM_TILE_ZOOM_LEVEL_RANGE = (3, 19)
WHEEL_ZOOM_LEVEL_RANGE = (3, 24)
VEHICLE_OVERLAY_OPACITY = 0.4


@dataclass
class RoutePoint:
    """A planned route point in local ENU meters."""

    x: float
    y: float
    z: float = 0.0
    speed: float = 1.0


@dataclass
class ZigZagArea:
    """Editable rectangle that defines a generated zig-zag route."""

    center_x: float
    center_y: float
    length: float
    width: float
    yaw: float = 0.0
    lane_spacing: float = 2.0
    turn_radius: float = 1.0
    start_x_sign: float = -1.0
    start_y_sign: float = -1.0


class RouteMapCanvas(QWidget):
    """Metric ENU map canvas with pan/zoom and waypoint editing."""

    route_changed = pyqtSignal()
    hover_changed = pyqtSignal(float, float)
    zoom_changed = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMinimumSize(120, 420)

        self.points = []
        self.planning_enabled = False
        self.map_source = 'OpenStreetMap'
        self.px_per_meter = 8.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.dragging_view = False
        self.dragging_point_index = None
        self.last_mouse_pos = None
        self.hover_world = RoutePoint(0.0, 0.0)
        self.default_point_z = 0.0
        self.default_point_speed = 1.0
        self.vehicle_pose = None
        self.vehicle_overlay_model = None
        self.visual_markers = []
        self.active_tool = None
        self.manual_points = []
        self.zigzag_areas = []
        self.route_items = []
        self.selected_zigzag_index = None
        self.default_zigzag_lane_spacing = 2.0
        self.default_zigzag_turn_radius = 1.0
        self.zigzag_drag_mode = None
        self.zigzag_drag_start = None
        self.zigzag_area_start = None
        self.zigzag_resize_anchor = None
        self.zigzag_resize_handle = None
        self.rotate_cursor = self._make_rotate_cursor()
        self.enuref = [57.708870, 11.974560, 0.0]
        self.osm_tiles = OsmTileClient(self)
        self.osm_tiles.tile_ready.connect(self.update)

    def _make_rotate_cursor(self):
        pixmap = QPixmap(32, 32)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing, True)
        self._draw_circular_arrow(
            painter,
            QPointF(16.0, 16.0),
            9.0,
            45.0,
            -270.0,
            QColor('#111827'),
            2,
            7.0,
            7.0,
        )
        painter.end()
        return QCursor(pixmap, 16, 16)

    @staticmethod
    def _arc_point(center, radius, angle_degrees):
        angle = math.radians(angle_degrees)
        return QPointF(
            center.x() + math.cos(angle) * radius,
            center.y() - math.sin(angle) * radius,
        )

    @classmethod
    def _draw_circular_arrow(
        cls,
        painter,
        center,
        radius,
        start_angle_degrees,
        sweep_angle_degrees,
        color,
        pen_width,
        arrow_length,
        arrow_width,
    ):
        rect = QRectF(
            center.x() - radius,
            center.y() - radius,
            radius * 2.0,
            radius * 2.0,
        )
        painter.setPen(QPen(color, pen_width))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(rect, int(start_angle_degrees * 16), int(sweep_angle_degrees * 16))

        arrow_angle = start_angle_degrees + sweep_angle_degrees
        angle = math.radians(arrow_angle)
        tip = cls._arc_point(center, radius, arrow_angle)
        tangent_x = -math.sin(angle)
        tangent_y = -math.cos(angle)
        if sweep_angle_degrees < 0.0:
            tangent_x = -tangent_x
            tangent_y = -tangent_y

        base = QPointF(
            tip.x() - tangent_x * arrow_length,
            tip.y() - tangent_y * arrow_length,
        )
        normal_x = -tangent_y
        normal_y = tangent_x
        half_width = arrow_width * 0.5
        painter.setBrush(color)
        painter.drawPolygon(
            QPolygonF(
                [
                    tip,
                    QPointF(
                        base.x() + normal_x * half_width,
                        base.y() + normal_y * half_width,
                    ),
                    QPointF(
                        base.x() - normal_x * half_width,
                        base.y() - normal_y * half_width,
                    ),
                ]
            )
        )

    def set_map_source(self, source):
        """Select the map background source."""
        if source not in ('OpenStreetMap', 'Local OSM server', 'None'):
            return
        self.map_source = source
        if self.map_source in ('OpenStreetMap', 'Local OSM server'):
            self.osm_tiles.refresh()
        self.update()

    def set_tile_server_url(self, url):
        self.osm_tiles.set_tile_server_url(url)

    def set_tile_cache_dir(self, cache_dir):
        self.osm_tiles.set_cache_dir(cache_dir)

    def refresh_tiles(self, clear_disk=False):
        self.osm_tiles.refresh(clear_disk=clear_disk)
        self.update()

    def set_planning_enabled(self, enabled):
        self.planning_enabled = bool(enabled)
        if not self.planning_enabled:
            self.set_active_tool(None)
        self.update()

    def set_active_tool(self, tool_name):
        """Select an optional route generation tool."""
        self.active_tool = tool_name if tool_name == 'zigzag' else None
        self.zigzag_drag_mode = None
        if self.active_tool != 'zigzag':
            self.selected_zigzag_index = None
        self.update()

    def set_default_point_values(self, z=None, speed=None):
        if z is not None:
            self.default_point_z = float(z)
        if speed is not None:
            self.default_point_speed = float(speed)

    def set_route_points(self, points):
        self.points = [
            RoutePoint(
                float(point.x),
                float(point.y),
                float(getattr(point, 'z', 0.0)),
                float(getattr(point, 'speed', 1.0)),
            )
            for point in points
        ]
        self.manual_points = list(self.points)
        self.zigzag_areas.clear()
        self.route_items = [('manual', index) for index in range(len(self.manual_points))]
        self.selected_zigzag_index = None
        self.route_changed.emit()
        self.update()

    def get_route_points(self):
        return list(self.points)

    def clear_route(self):
        self.points.clear()
        self.manual_points.clear()
        self.zigzag_areas.clear()
        self.route_items.clear()
        self.selected_zigzag_index = None
        self.route_changed.emit()
        self.update()

    def set_vehicle_pose(self, x, y, yaw_rad=0.0):
        self.vehicle_pose = (float(x), float(y), float(yaw_rad))
        self.update()

    def set_vehicle_overlay_model(self, model):
        self.vehicle_overlay_model = model
        self.update()

    def set_visual_markers(self, markers):
        self.visual_markers = list(markers)
        self.update()

    def set_enu_ref(self, enuref):
        if len(enuref) < 3:
            return

        is_zero = (
            float(enuref[0]) == 0.0 and float(enuref[1]) == 0.0 and float(enuref[2]) == 0.0
        )
        is_default = (
            abs(float(enuref[0]) - 57.708870) < 1e-6
            and abs(float(enuref[1]) - 11.974560) < 1e-6
        )
        has_connected_vehicle = self.enuref != [0.0, 0.0, 0.0] and not (
            abs(self.enuref[0] - 57.708870) < 1e-6
            and abs(self.enuref[1] - 11.974560) < 1e-6
        )

        if has_connected_vehicle and (is_zero or is_default):
            return

        self.enuref = [float(enuref[0]), float(enuref[1]), float(enuref[2])]
        self.update()

    def fit_route(self):
        bounds_points = list(self.points)
        if self.vehicle_pose is not None:
            bounds_points.append(RoutePoint(self.vehicle_pose[0], self.vehicle_pose[1]))
        if not bounds_points:
            self.center_x = 0.0
            self.center_y = 0.0
            self.px_per_meter = 8.0
            self.update()
            return

        min_x = min(point.x for point in bounds_points)
        max_x = max(point.x for point in bounds_points)
        min_y = min(point.y for point in bounds_points)
        max_y = max(point.y for point in bounds_points)
        self.center_x = (min_x + max_x) * 0.5
        self.center_y = (min_y + max_y) * 0.5

        width = max(max_x - min_x, 1.0)
        height = max(max_y - min_y, 1.0)
        self.px_per_meter = max(
            0.5,
            min(35.0, min((self.width() - 80.0) / width, (self.height() - 80.0) / height)),
        )
        self.zoom_changed.emit(self._osm_zoom_level())
        self.update()

    def center_on(self, x, y):
        """Pan the map center without changing the current zoom."""
        self.center_x = float(x)
        self.center_y = float(y)
        self.update()

    def center_on_vehicle(self):
        if self.vehicle_pose is None:
            return False
        self.center_on(self.vehicle_pose[0], self.vehicle_pose[1])
        return True

    def center_on_home(self):
        self.center_on(0.0, 0.0)
        return True

    def screen_to_world(self, pos):
        x = self.center_x + (pos.x() - self.width() * 0.5) / self.px_per_meter
        y = self.center_y - (pos.y() - self.height() * 0.5) / self.px_per_meter
        return RoutePoint(x, y, self.default_point_z, self.default_point_speed)

    def world_to_screen(self, point):
        x = self.width() * 0.5 + (point.x - self.center_x) * self.px_per_meter
        y = self.height() * 0.5 - (point.y - self.center_y) * self.px_per_meter
        return QPointF(x, y)

    def nearest_point_index(self, world_point, max_px=18.0):
        if not self.points:
            return None
        max_distance_m = max_px / self.px_per_meter
        closest_index = None
        closest_distance = max_distance_m
        for index, point in enumerate(self.points):
            distance = math.hypot(point.x - world_point.x, point.y - world_point.y)
            if distance <= closest_distance:
                closest_distance = distance
                closest_index = index
        return closest_index

    def insertion_index(self, world_point):
        if len(self.points) < 2:
            return len(self.points)

        best_index = len(self.points)
        best_distance = float('inf')
        for index in range(len(self.points) - 1):
            distance = self._distance_to_segment(
                world_point, self.points[index], self.points[index + 1]
            )
            if distance < best_distance:
                best_distance = distance
                best_index = index + 1

        first_distance = math.hypot(
            world_point.x - self.points[0].x, world_point.y - self.points[0].y
        )
        last_distance = math.hypot(
            world_point.x - self.points[-1].x, world_point.y - self.points[-1].y
        )
        if first_distance <= best_distance:
            return 0
        if last_distance <= best_distance:
            return len(self.points)
        return best_index

    @staticmethod
    def _distance_to_segment(point, start, end):
        vx = end.x - start.x
        vy = end.y - start.y
        wx = point.x - start.x
        wy = point.y - start.y
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-9:
            return math.hypot(point.x - start.x, point.y - start.y)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
        projection_x = start.x + t * vx
        projection_y = start.y + t * vy
        return math.hypot(point.x - projection_x, point.y - projection_y)

    def mousePressEvent(self, event):
        self.setFocus()
        self.last_mouse_pos = event.pos()
        world_point = self.screen_to_world(event.pos())

        if (
            self.planning_enabled
            and self.active_tool == 'zigzag'
            and event.button() in (Qt.LeftButton, Qt.RightButton)
        ):
            self._zigzag_mouse_press(world_point, event)
            return

        if self.planning_enabled and event.button() == Qt.LeftButton:
            if event.modifiers() & Qt.ControlModifier:
                self.manual_points.append(world_point)
                self.route_items.append(('manual', len(self.manual_points) - 1))
                self._rebuild_route_points()
                self.route_changed.emit()
                self.dragging_point_index = None
                self.update()
                return
            if event.modifiers() & Qt.ShiftModifier:
                index = self._nearest_manual_point_index(world_point)
                if index is not None:
                    self.dragging_point_index = index
                self.update()
                return

        if self.planning_enabled and event.button() == Qt.RightButton:
            index = self._nearest_manual_point_index(world_point)
            if index is not None:
                self.manual_points.pop(index)
                self._remove_route_item('manual', index)
                self._rebuild_route_points()
                self.route_changed.emit()
                self.update()
            return

        if event.button() in (Qt.LeftButton, Qt.MiddleButton):
            self.dragging_view = True

    def mouseMoveEvent(self, event):
        world_point = self.screen_to_world(event.pos())
        self.hover_world = world_point
        self.hover_changed.emit(world_point.x, world_point.y)

        if self.active_tool == 'zigzag' and self.zigzag_drag_mode is not None:
            self._zigzag_mouse_move(world_point)
            return

        if self.active_tool == 'zigzag':
            self._update_zigzag_cursor(world_point)

        if self.dragging_point_index is not None:
            existing_point = self.manual_points[self.dragging_point_index]
            existing_point.x = world_point.x
            existing_point.y = world_point.y
            self._rebuild_route_points()
            self.route_changed.emit()
            self.update()
            return

        if self.dragging_view and self.last_mouse_pos is not None:
            delta = event.pos() - self.last_mouse_pos
            self.center_x -= delta.x() / self.px_per_meter
            self.center_y += delta.y() / self.px_per_meter
            self.last_mouse_pos = event.pos()
            self.update()
            return

        self.update()

    def mouseReleaseEvent(self, event):
        if self.active_tool == 'zigzag' and self.zigzag_drag_mode is not None:
            self._zigzag_mouse_release()
            return

        self.dragging_view = False
        self.dragging_point_index = None
        self.last_mouse_pos = None

    def leaveEvent(self, event):
        self.unsetCursor()
        self.hover_changed.emit(float('nan'), float('nan'))
        super().leaveEvent(event)

    def wheelEvent(self, event):
        before = self.screen_to_world(event.pos())
        zoom_factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15

        lat0 = max(min(float(self.enuref[0]), 85.0), -85.0)
        meters_per_pixel_at_zoom_0 = 156543.03392 * max(math.cos(math.radians(lat0)), 0.01)
        min_zoom, max_zoom = WHEEL_ZOOM_LEVEL_RANGE
        min_px_per_meter = (1 << min_zoom) / meters_per_pixel_at_zoom_0
        max_px_per_meter = (1 << max_zoom) / meters_per_pixel_at_zoom_0

        self.px_per_meter = max(
            min_px_per_meter,
            min(max_px_per_meter, self.px_per_meter * zoom_factor),
        )
        after = self.screen_to_world(event.pos())
        self.center_x += before.x - after.x
        self.center_y += before.y - after.y

        world_point = self.screen_to_world(event.pos())
        self.hover_world = world_point
        self.hover_changed.emit(world_point.x, world_point.y)
        self.zoom_changed.emit(self._osm_zoom_level())
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor('#d1d5db'))

        if self.map_source in ('OpenStreetMap', 'Local OSM server'):
            self._draw_osm_tiles(painter)

        self._draw_grid(painter)
        self._draw_zigzag_area(painter)
        self._draw_route(painter)
        self._draw_visual_markers(painter)
        self._draw_vehicle(painter)

    def _draw_osm_tiles(self, painter):
        zoom = self._osm_zoom_level()
        corners = [
            self.screen_to_world(QPointF(0.0, 0.0)),
            self.screen_to_world(QPointF(float(self.width()), 0.0)),
            self.screen_to_world(QPointF(0.0, float(self.height()))),
            self.screen_to_world(QPointF(float(self.width()), float(self.height()))),
        ]
        lat_lons = [enu_to_llh(point.x, point.y, self.enuref)[:2] for point in corners]
        min_lat = min(lat for lat, _ in lat_lons)
        max_lat = max(lat for lat, _ in lat_lons)
        min_lon = min(lon for _, lon in lat_lons)
        max_lon = max(lon for _, lon in lat_lons)

        max_tile = (1 << zoom) - 1
        x_min = max(0, min(max_tile, lon_to_tile_x(min_lon, zoom)))
        x_max = max(0, min(max_tile, lon_to_tile_x(max_lon, zoom)))
        y_min = max(0, min(max_tile, lat_to_tile_y(max_lat, zoom)))
        y_max = max(0, min(max_tile, lat_to_tile_y(min_lat, zoom)))

        if x_max < x_min:
            x_min, x_max = x_max, x_min
        if y_max < y_min:
            y_min, y_max = y_max, y_min

        if (x_max - x_min + 1) * (y_max - y_min + 1) > 120:
            painter.fillRect(self.rect(), QColor('#e5e7eb'))
            painter.setPen(QColor('#4b5563'))
            painter.drawText(12, self.height() - 14, 'Zoom in to load OSM tiles')
            return

        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        for tile_y in range(y_min, y_max + 1):
            for tile_x in range(x_min, x_max + 1):
                north = tile_y_to_lat(tile_y, zoom)
                south = tile_y_to_lat(tile_y + 1, zoom)
                west = tile_x_to_lon(tile_x, zoom)
                east = tile_x_to_lon(tile_x + 1, zoom)
                west_x, north_y = llh_to_enu(north, west, self.enuref)
                east_x, south_y = llh_to_enu(south, east, self.enuref)
                top_left = self.world_to_screen(RoutePoint(west_x, north_y))
                bottom_right = self.world_to_screen(RoutePoint(east_x, south_y))
                rect = QRectF(top_left, bottom_right).normalized()

                pixmap = self.osm_tiles.get_tile(zoom, tile_x, tile_y)
                if pixmap is None:
                    draw_tile_placeholder(painter, rect, 'Loading OSM')
                else:
                    painter.drawPixmap(rect, pixmap, QRectF(pixmap.rect()))

        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)

    def _osm_zoom_level(self):
        lat0 = max(min(float(self.enuref[0]), 85.0), -85.0)
        scale = 156543.03392 * max(math.cos(math.radians(lat0)), 0.01) * self.px_per_meter
        min_zoom, max_zoom = OSM_TILE_ZOOM_LEVEL_RANGE
        return max(min_zoom, min(max_zoom, int(round(math.log(max(scale, 1.0), 2)))))

    def _draw_grid(self, painter):
        grid_spacing_m = self._grid_spacing()
        left = self.center_x - self.width() * 0.5 / self.px_per_meter
        right = self.center_x + self.width() * 0.5 / self.px_per_meter
        bottom = self.center_y - self.height() * 0.5 / self.px_per_meter
        top = self.center_y + self.height() * 0.5 / self.px_per_meter

        painter.setPen(QPen(QColor(31, 41, 55, 80), 1))
        x = math.floor(left / grid_spacing_m) * grid_spacing_m
        while x <= right:
            sx = self.world_to_screen(RoutePoint(x, 0.0)).x()
            painter.drawLine(int(sx), 0, int(sx), self.height())
            x += grid_spacing_m

        y = math.floor(bottom / grid_spacing_m) * grid_spacing_m
        while y <= top:
            sy = self.world_to_screen(RoutePoint(0.0, y)).y()
            painter.drawLine(0, int(sy), self.width(), int(sy))
            y += grid_spacing_m

        painter.setPen(QPen(QColor(31, 41, 55, 150), 2))
        origin = self.world_to_screen(RoutePoint(0.0, 0.0))
        painter.drawLine(int(origin.x()), 0, int(origin.x()), self.height())
        painter.drawLine(0, int(origin.y()), self.width(), int(origin.y()))

    def _grid_spacing(self):
        target_px = 80.0
        raw_spacing = target_px / self.px_per_meter
        exponent = math.floor(math.log10(max(raw_spacing, 1e-6)))
        base = 10**exponent
        for multiplier in (1, 2, 5, 10):
            spacing = multiplier * base
            if spacing * self.px_per_meter >= target_px:
                return spacing
        return 10 * base

    def _draw_route(self, painter):
        if len(self.points) > 1:
            painter.setPen(QPen(QColor('#facc15'), 3))
            for start, end in zip(self.points[:-1], self.points[1:]):
                painter.drawLine(self.world_to_screen(start), self.world_to_screen(end))

        font = QFont('Sans Serif', 9)
        font.setBold(True)
        painter.setFont(font)
        for index, point in enumerate(self.points, start=1):
            screen = self.world_to_screen(point)
            label = str(index)
            metrics = painter.fontMetrics()
            label_rect = metrics.boundingRect(label)
            radius = max(7.0, label_rect.width() * 0.5 + 4.0)
            painter.setPen(QPen(QColor('#92400e'), 3))
            painter.setBrush(QColor('#fde047'))
            painter.drawEllipse(screen, radius, 7.0)
            painter.setPen(QColor('#111827'))
            painter.drawText(
                QRectF(screen.x() - radius, screen.y() - 7.0, radius * 2.0, 14.0),
                Qt.AlignCenter,
                label,
            )

        painter.setOpacity(0.55)
        if len(self.points) >= 2:
            p0, p1 = self.points[0], self.points[1]
            yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
            center = self.world_to_screen(p0)
            tip = center + QPointF(math.cos(yaw) * 14.0, -math.sin(yaw) * 14.0)
            left = center + QPointF(math.cos(yaw + 2.45) * 9.0, -math.sin(yaw + 2.45) * 9.0)
            right = center + QPointF(math.cos(yaw - 2.45) * 9.0, -math.sin(yaw - 2.45) * 9.0)
            painter.setPen(QPen(QColor('#14532d'), 2))
            painter.setBrush(QColor('#22c55e'))
            painter.drawPolygon(QPolygonF([tip, left, right]))

        if self.points:
            screen = self.world_to_screen(self.points[-1])
            half = 7.0
            painter.setPen(QPen(QColor('#7f1d1d'), 2))
            painter.setBrush(QColor('#dc2626'))
            painter.drawRect(QRectF(screen.x() - half, screen.y() - half, half * 2, half * 2))
        painter.setOpacity(1.0)

    def _nearest_manual_point_index(self, world_point, max_px=18.0):
        if not self.manual_points:
            return None
        max_distance_m = max_px / self.px_per_meter
        closest_index = None
        closest_distance = max_distance_m
        for index, point in enumerate(self.manual_points):
            distance = math.hypot(point.x - world_point.x, point.y - world_point.y)
            if distance <= closest_distance:
                closest_distance = distance
                closest_index = index
        return closest_index

    def _zigzag_mouse_press(self, world_point, event):
        area_index = self._zigzag_area_at(world_point)
        if event.button() == Qt.RightButton:
            if area_index is not None:
                self.selected_zigzag_index = area_index
                self._show_zigzag_context_menu(area_index, event.globalPos())
            return

        handle = self._zigzag_hit_test(world_point)
        self.zigzag_drag_start = RoutePoint(world_point.x, world_point.y)
        self.zigzag_area_start = None

        if handle is not None:
            self.selected_zigzag_index, self.zigzag_drag_mode = handle
            self.zigzag_area_start = self._copy_zigzag_area()
            if self.zigzag_drag_mode.startswith('resize'):
                self.zigzag_resize_handle = self.zigzag_drag_mode.split(':', 1)[1]
                self.zigzag_resize_anchor = self._zigzag_opposite_local_corner_or_edge(
                    self.zigzag_resize_handle
                )
            return

        if area_index is not None:
            self.selected_zigzag_index = area_index
            self.zigzag_area_start = self._copy_zigzag_area()
            self.zigzag_drag_mode = 'move'
            return

        self.zigzag_areas.append(
            ZigZagArea(
                world_point.x,
                world_point.y,
                0.1,
                0.1,
                0.0,
                self.default_zigzag_lane_spacing,
                self.default_zigzag_turn_radius,
            )
        )
        self.selected_zigzag_index = len(self.zigzag_areas) - 1
        self.route_items.append(('zigzag', self.selected_zigzag_index))
        self.zigzag_drag_mode = 'draw'
        self.zigzag_area_start = self._copy_zigzag_area()
        self._update_zigzag_routes()

    def _zigzag_mouse_move(self, world_point):
        area = self._selected_zigzag_area()
        if self.zigzag_drag_start is None or area is None:
            return

        if self.zigzag_drag_mode == 'draw':
            start = self.zigzag_drag_start
            area.center_x = (start.x + world_point.x) * 0.5
            area.center_y = (start.y + world_point.y) * 0.5
            area.length = max(abs(world_point.x - start.x), 0.1)
            area.width = max(abs(world_point.y - start.y), 0.1)
            area.start_x_sign = -1.0 if start.x <= world_point.x else 1.0
            area.start_y_sign = -1.0 if start.y <= world_point.y else 1.0
        elif self.zigzag_drag_mode == 'move' and self.zigzag_area_start is not None:
            area.center_x = (
                self.zigzag_area_start.center_x + world_point.x - self.zigzag_drag_start.x
            )
            area.center_y = (
                self.zigzag_area_start.center_y + world_point.y - self.zigzag_drag_start.y
            )
        elif self.zigzag_drag_mode == 'rotate':
            area.yaw = math.atan2(
                world_point.y - area.center_y,
                world_point.x - area.center_x,
            ) - math.pi * 0.5
        elif self.zigzag_drag_mode.startswith('resize') and self.zigzag_resize_anchor:
            self._resize_zigzag_area(world_point)

        self._update_zigzag_routes()

    def _zigzag_mouse_release(self):
        self.zigzag_drag_mode = None
        self.zigzag_drag_start = None
        self.zigzag_area_start = None
        self.zigzag_resize_anchor = None
        self.zigzag_resize_handle = None
        self.dragging_view = False
        self.last_mouse_pos = None
        self.update()

    def _copy_zigzag_area(self):
        area = self._selected_zigzag_area()
        if area is None:
            return None
        return ZigZagArea(
            area.center_x,
            area.center_y,
            area.length,
            area.width,
            area.yaw,
            area.lane_spacing,
            area.turn_radius,
            area.start_x_sign,
            area.start_y_sign,
        )

    def _resize_zigzag_area(self, world_point):
        area = self._selected_zigzag_area()
        start_area = self.zigzag_area_start
        if area is None or start_area is None:
            return
        local = self._zigzag_world_to_local(world_point, start_area)
        anchor_x, anchor_y = self.zigzag_resize_anchor
        handle = self.zigzag_resize_handle

        changes_x = 'right' in handle or 'left' in handle
        changes_y = 'top' in handle or 'bottom' in handle
        new_x = local.x if changes_x else anchor_x + start_area.length
        new_y = local.y if changes_y else anchor_y + start_area.width
        min_size = 0.1
        length = max(abs(new_x - anchor_x), min_size) if changes_x else start_area.length
        width = max(abs(new_y - anchor_y), min_size) if changes_y else start_area.width
        center_local_x = (new_x + anchor_x) * 0.5 if changes_x else 0.0
        center_local_y = (new_y + anchor_y) * 0.5 if changes_y else 0.0
        center_world = self._zigzag_local_to_world(center_local_x, center_local_y, start_area)

        area.center_x = center_world.x
        area.center_y = center_world.y
        area.length = length
        area.width = width

    def _zigzag_opposite_local_corner_or_edge(self, handle):
        area = self._selected_zigzag_area()
        if area is None:
            return None
        x = area.length * 0.5
        y = area.width * 0.5
        if 'left' in handle:
            anchor_x = x
        elif 'right' in handle:
            anchor_x = -x
        elif 'top' in handle or 'bottom' in handle:
            anchor_x = -x
        else:
            anchor_x = 0.0

        if 'top' in handle:
            anchor_y = -y
        elif 'bottom' in handle:
            anchor_y = y
        elif 'left' in handle or 'right' in handle:
            anchor_y = -y
        else:
            anchor_y = 0.0
        return (anchor_x, anchor_y)

    def _update_zigzag_cursor(self, world_point):
        hit = self._zigzag_hit_test(world_point)
        if hit is None:
            if self._zigzag_area_at(world_point) is not None:
                self.setCursor(Qt.SizeAllCursor)
            else:
                self.unsetCursor()
            return

        _area_index, mode = hit
        if mode == 'rotate':
            self.setCursor(self.rotate_cursor)
        elif mode in ('resize:left', 'resize:right'):
            self.setCursor(Qt.SizeHorCursor)
        elif mode in ('resize:top', 'resize:bottom'):
            self.setCursor(Qt.SizeVerCursor)
        elif mode in ('resize:top_left', 'resize:bottom_right'):
            self.setCursor(Qt.SizeFDiagCursor)
        elif mode in ('resize:top_right', 'resize:bottom_left'):
            self.setCursor(Qt.SizeBDiagCursor)
        else:
            self.unsetCursor()

    def _zigzag_hit_test(self, world_point):
        handles = []
        for area_index in range(len(self.zigzag_areas) - 1, -1, -1):
            handles.extend(
                (area_index, name, point)
                for name, point in self._zigzag_handle_points(area_index)
            )
        max_distance = 14.0 / self.px_per_meter
        for area_index, name, point in handles:
            if math.hypot(world_point.x - point.x, world_point.y - point.y) <= max_distance:
                if name == 'rotate':
                    return (area_index, 'rotate')
                return (area_index, f'resize:{name}')

        for area_index in range(len(self.zigzag_areas) - 1, -1, -1):
            edge = self._zigzag_edge_hit_test(world_point, area_index)
            if edge is not None:
                return (area_index, f'resize:{edge}')
        return None

    def _zigzag_edge_hit_test(self, world_point, area_index):
        area = self.zigzag_areas[area_index]
        local = self._zigzag_world_to_local(world_point, area)
        half_l = area.length * 0.5
        half_w = area.width * 0.5
        tolerance = 10.0 / self.px_per_meter

        within_x = -half_l - tolerance <= local.x <= half_l + tolerance
        within_y = -half_w - tolerance <= local.y <= half_w + tolerance
        if not (within_x and within_y):
            return None

        distances = [
            ('left', abs(local.x + half_l)),
            ('right', abs(local.x - half_l)),
            ('bottom', abs(local.y + half_w)),
            ('top', abs(local.y - half_w)),
        ]
        edge, distance = min(distances, key=lambda item: item[1])
        if distance <= tolerance:
            return edge
        return None

    def _zigzag_handle_points(self, area_index=None):
        area = self._selected_zigzag_area() if area_index is None else self.zigzag_areas[area_index]
        if area is None:
            return []
        half_l = area.length * 0.5
        half_w = area.width * 0.5
        local_handles = [
            ('top_left', -half_l, half_w),
            ('top', 0.0, half_w),
            ('top_right', half_l, half_w),
            ('right', half_l, 0.0),
            ('bottom_right', half_l, -half_w),
            ('bottom', 0.0, -half_w),
            ('bottom_left', -half_l, -half_w),
            ('left', -half_l, 0.0),
            ('rotate', 0.0, half_w + max(24.0 / self.px_per_meter, 0.5)),
        ]
        return [
            (name, self._zigzag_local_to_world(x, y, area))
            for name, x, y in local_handles
        ]

    def _zigzag_area_at(self, world_point):
        for area_index in range(len(self.zigzag_areas) - 1, -1, -1):
            area = self.zigzag_areas[area_index]
            local = self._zigzag_world_to_local(world_point, area)
            if abs(local.x) <= area.length * 0.5 and abs(local.y) <= area.width * 0.5:
                return area_index
        return None

    def _zigzag_world_to_local(self, world_point, area=None):
        area = area or self._selected_zigzag_area()
        dx = world_point.x - area.center_x
        dy = world_point.y - area.center_y
        cos_yaw = math.cos(area.yaw)
        sin_yaw = math.sin(area.yaw)
        return RoutePoint(dx * cos_yaw + dy * sin_yaw, -dx * sin_yaw + dy * cos_yaw)

    def _zigzag_local_to_world(self, x, y, area=None):
        area = area or self._selected_zigzag_area()
        cos_yaw = math.cos(area.yaw)
        sin_yaw = math.sin(area.yaw)
        return RoutePoint(
            area.center_x + x * cos_yaw - y * sin_yaw,
            area.center_y + x * sin_yaw + y * cos_yaw,
            self.default_point_z,
            self.default_point_speed,
        )

    def _zigzag_corners(self, area_index=None):
        area = self._selected_zigzag_area() if area_index is None else self.zigzag_areas[area_index]
        if area is None:
            return []
        half_l = area.length * 0.5
        half_w = area.width * 0.5
        return [
            self._zigzag_local_to_world(-half_l, -half_w, area),
            self._zigzag_local_to_world(half_l, -half_w, area),
            self._zigzag_local_to_world(half_l, half_w, area),
            self._zigzag_local_to_world(-half_l, half_w, area),
        ]

    def _update_zigzag_routes(self):
        self._rebuild_route_points()
        self.route_changed.emit()
        self.update()

    def _rebuild_route_points(self):
        points = []
        for item_type, index in self.route_items:
            if item_type == 'manual' and 0 <= index < len(self.manual_points):
                points.append(self.manual_points[index])
            elif item_type == 'zigzag' and 0 <= index < len(self.zigzag_areas):
                points.extend(self._generate_zigzag_points(self.zigzag_areas[index]))
        self.points = points

    def _remove_route_item(self, item_type, removed_index):
        updated_items = []
        for route_item_type, index in self.route_items:
            if route_item_type == item_type:
                if index == removed_index:
                    continue
                if index > removed_index:
                    index -= 1
            updated_items.append((route_item_type, index))
        self.route_items = updated_items

    def _generate_zigzag_points(self, area):
        if area is None or area.length < 0.1 or area.width < 0.1:
            return []

        lane_count = max(2, int(math.floor(area.width / area.lane_spacing)) + 1)
        if lane_count == 1:
            offsets = [0.0]
        else:
            actual_spacing = area.width / (lane_count - 1)
            offsets = [-area.width * 0.5 + index * actual_spacing for index in range(lane_count)]
        if area.start_y_sign > 0.0:
            offsets.reverse()

        half_l = area.length * 0.5
        points = []
        first_start_x = -half_l if area.start_x_sign < 0.0 else half_l
        first_end_x = -first_start_x
        for index, y in enumerate(offsets):
            start_x, end_x = (
                (first_start_x, first_end_x) if index % 2 == 0 else (first_end_x, first_start_x)
            )
            points.append(self._zigzag_local_to_world(start_x, y, area))
            points.append(self._zigzag_local_to_world(end_x, y, area))

            if index < len(offsets) - 1:
                next_y = offsets[index + 1]
                points.extend(self._zigzag_turn_points(end_x, y, next_y, area))

        return points

    def _zigzag_turn_points(self, x_edge, y0, y1, area):
        radius = min(area.turn_radius, area.length * 0.5)
        if radius <= 1e-6:
            return []

        direction = 1.0 if x_edge >= 0.0 else -1.0
        steps = max(3, int(radius / max(area.lane_spacing, 0.1) * 6.0))
        turn_points = []
        for step in range(1, steps + 1):
            t = step / (steps + 1)
            y = y0 + (y1 - y0) * t
            bulge = math.sin(math.pi * t) * radius
            x = x_edge + direction * bulge
            turn_points.append(self._zigzag_local_to_world(x, y, area))

        return turn_points

    def _draw_zigzag_area(self, painter):
        if not self.zigzag_areas or self.active_tool != 'zigzag':
            return

        painter.save()
        for area_index, area in enumerate(self.zigzag_areas):
            corners = self._zigzag_corners(area_index)
            if len(corners) < 4:
                continue

            polygon = QPolygonF([self.world_to_screen(point) for point in corners])
            painter.setPen(QPen(QColor('#0f766e'), 2, Qt.DashLine))
            painter.setBrush(QColor(20, 184, 166, 38))
            painter.drawPolygon(polygon)

            for name, point in self._zigzag_handle_points(area_index):
                screen = self.world_to_screen(point)
                if name == 'rotate':
                    painter.setPen(QPen(QColor('#7c2d12'), 2))
                    painter.setBrush(QColor('#dc2626'))
                    painter.drawEllipse(screen, 6.0, 6.0)
                    self._draw_rotate_handle_arrow(painter, screen)
                else:
                    painter.setPen(QPen(QColor('#064e3b'), 1))
                    painter.setBrush(QColor('#ccfbf1'))
                    painter.drawRect(QRectF(screen.x() - 4.0, screen.y() - 4.0, 8.0, 8.0))
        painter.restore()

    def _draw_rotate_handle_arrow(self, painter, center):
        self._draw_circular_arrow(
            painter,
            center,
            14.0,
            45.0,
            -270.0,
            QColor('#7c2d12'),
            2,
            8.0,
            8.0,
        )

    def _selected_zigzag_area(self):
        if self.selected_zigzag_index is None:
            return None
        if self.selected_zigzag_index < 0 or self.selected_zigzag_index >= len(self.zigzag_areas):
            return None
        return self.zigzag_areas[self.selected_zigzag_index]

    def _show_zigzag_context_menu(self, area_index, global_pos):
        area = self.zigzag_areas[area_index]
        menu = QMenu(self)
        lane_spin = self._create_context_spin_box(area.lane_spacing, 0.10, 1000.0, 0.25)
        turn_spin = self._create_context_spin_box(area.turn_radius, 0.0, 1000.0, 0.25)
        menu.addAction(self._create_spin_action(menu, 'Lane spacing [m]', lane_spin))
        menu.addAction(self._create_spin_action(menu, 'Turn radius [m]', turn_spin))
        menu.addSeparator()
        delete_action = menu.addAction('Delete')

        lane_spin.valueChanged.connect(
            lambda value, index=area_index: self._set_zigzag_lane_spacing(index, value)
        )
        turn_spin.valueChanged.connect(
            lambda value, index=area_index: self._set_zigzag_turn_radius(index, value)
        )

        selected_action = menu.exec_(global_pos)
        if selected_action == delete_action:
            self.zigzag_areas.pop(area_index)
            self._remove_route_item('zigzag', area_index)
            self.selected_zigzag_index = None
            self._update_zigzag_routes()

    def _create_context_spin_box(self, value, minimum, maximum, step):
        spin_box = QDoubleSpinBox()
        spin_box.setDecimals(2)
        spin_box.setMinimum(minimum)
        spin_box.setMaximum(maximum)
        spin_box.setSingleStep(step)
        spin_box.setValue(value)
        return spin_box

    def _create_spin_action(self, menu, label_text, spin_box):
        widget = QWidget(menu)
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(8, 4, 8, 4)
        label = QLabel(label_text, widget)
        layout.addWidget(label)
        layout.addWidget(spin_box)
        action = QWidgetAction(menu)
        action.setDefaultWidget(widget)
        return action

    def _set_zigzag_lane_spacing(self, area_index, value):
        if 0 <= area_index < len(self.zigzag_areas):
            self.zigzag_areas[area_index].lane_spacing = max(0.1, float(value))
            self.default_zigzag_lane_spacing = self.zigzag_areas[area_index].lane_spacing
            self._update_zigzag_routes()

    def _set_zigzag_turn_radius(self, area_index, value):
        if 0 <= area_index < len(self.zigzag_areas):
            self.zigzag_areas[area_index].turn_radius = max(0.0, float(value))
            self.default_zigzag_turn_radius = self.zigzag_areas[area_index].turn_radius
            self._update_zigzag_routes()

    def _draw_vehicle(self, painter):
        if self.vehicle_pose is None:
            return

        x, y, yaw = self.vehicle_pose
        center = self.world_to_screen(RoutePoint(x, y))

        self._draw_vehicle_urdf_overlay(painter, x, y, yaw)

        painter.setPen(QPen(QColor('#7f1d1d'), 2))
        painter.setBrush(QColor('#dc2626'))
        painter.drawEllipse(center, 5.0, 5.0)

        heading = QPointF(math.cos(yaw) * 18.0, -math.sin(yaw) * 18.0)
        left = QPointF(math.cos(yaw + 2.5) * 10.0, -math.sin(yaw + 2.5) * 10.0)
        right = QPointF(math.cos(yaw - 2.5) * 10.0, -math.sin(yaw - 2.5) * 10.0)

        painter.setPen(QPen(QColor(56, 189, 248, 150), 2))
        painter.setBrush(QColor(14, 165, 233, 102))
        painter.drawPolygon(QPolygonF([center + heading, center + left, center + right]))

    def _draw_vehicle_urdf_overlay(self, painter, vehicle_x, vehicle_y, vehicle_yaw):
        if self.vehicle_overlay_model is None:
            return

        shapes = getattr(self.vehicle_overlay_model, 'shapes', [])
        if not shapes:
            return

        painter.save()
        painter.setOpacity(VEHICLE_OVERLAY_OPACITY)
        for shape in shapes:
            if shape.kind == 'polygon':
                polygon = QPolygonF(
                    [
                        self._vehicle_local_to_screen(vehicle_x, vehicle_y, vehicle_yaw, px, py)
                        for px, py in getattr(shape, 'points', [])
                    ]
                )
                if len(polygon) >= 3:
                    painter.setPen(QPen(QColor('#111827'), 1))
                    painter.setBrush(QColor(shape.color))
                    painter.drawPolygon(polygon)
            elif shape.kind == 'circle':
                center = self._vehicle_local_to_screen(
                    vehicle_x, vehicle_y, vehicle_yaw, shape.x, shape.y
                )
                radius = max(shape.width, shape.length) * 0.5 * self.px_per_meter
                painter.setPen(QPen(QColor('#111827'), 1))
                painter.setBrush(QColor(shape.color))
                painter.drawEllipse(center, radius, radius)
            else:
                polygon = self._vehicle_rect_to_screen_polygon(
                    vehicle_x,
                    vehicle_y,
                    vehicle_yaw,
                    shape.x,
                    shape.y,
                    shape.length,
                    shape.width,
                    shape.yaw,
                )
                painter.setPen(QPen(QColor('#111827'), 1))
                painter.setBrush(QColor(shape.color))
                painter.drawPolygon(polygon)
        painter.restore()

    def _draw_visual_markers(self, painter):
        for marker in self.visual_markers:
            action = getattr(marker, 'action', 0)
            if action not in (0, 1):
                continue

            marker_type = getattr(marker, 'type', -1)
            color = self._marker_color(marker)
            pose = getattr(marker, 'pose', None)
            if pose is None:
                continue

            if marker_type == 4:  # LINE_STRIP
                self._draw_marker_line_strip(painter, marker, color)
            elif marker_type in (1, 2, 3):  # CUBE, SPHERE, CYLINDER
                self._draw_marker_solid(painter, marker, pose, color)
            elif marker_type == 0:  # ARROW
                self._draw_marker_arrow(painter, marker, pose, color)
            elif marker_type == 9:  # TEXT_VIEW_FACING
                self._draw_marker_text(painter, marker, pose, color)

    def _draw_marker_line_strip(self, painter, marker, color):
        points = getattr(marker, 'points', [])
        if len(points) < 2:
            return
        width_px = max(1.0, getattr(marker.scale, 'x', 0.05) * self.px_per_meter)
        painter.setPen(QPen(color, width_px))
        previous = None
        for point in points:
            screen = self.world_to_screen(RoutePoint(point.x, point.y))
            if previous is not None:
                painter.drawLine(previous, screen)
            previous = screen

    def _draw_marker_solid(self, painter, marker, pose, color):
        center = self.world_to_screen(RoutePoint(pose.position.x, pose.position.y))
        width_px = max(4.0, getattr(marker.scale, 'x', 0.2) * self.px_per_meter)
        height_px = max(4.0, getattr(marker.scale, 'y', 0.2) * self.px_per_meter)
        painter.setPen(QPen(color.darker(130), 1))
        painter.setBrush(color)

        if marker.type in (2, 3):
            painter.drawEllipse(center, width_px * 0.5, height_px * 0.5)
            return

        yaw = self._yaw_from_orientation(pose.orientation)
        polygon = self._world_rect_to_screen_polygon(
            pose.position.x,
            pose.position.y,
            getattr(marker.scale, 'x', 0.2),
            getattr(marker.scale, 'y', 0.2),
            yaw,
        )
        painter.drawPolygon(polygon)

    def _draw_marker_arrow(self, painter, marker, pose, color):
        yaw = self._yaw_from_orientation(pose.orientation)
        length_px = max(14.0, getattr(marker.scale, 'x', 0.4) * self.px_per_meter)
        width_px = max(8.0, getattr(marker.scale, 'y', 0.2) * self.px_per_meter)
        center = self.world_to_screen(RoutePoint(pose.position.x, pose.position.y))
        tip = center + QPointF(math.cos(yaw) * length_px, -math.sin(yaw) * length_px)
        left = center + QPointF(
            math.cos(yaw + 2.45) * width_px,
            -math.sin(yaw + 2.45) * width_px,
        )
        right = center + QPointF(
            math.cos(yaw - 2.45) * width_px,
            -math.sin(yaw - 2.45) * width_px,
        )
        painter.setPen(QPen(color.darker(130), 1))
        painter.setBrush(color)
        painter.drawPolygon(QPolygonF([tip, left, right]))

    def _draw_marker_text(self, painter, marker, pose, color):
        text = getattr(marker, 'text', '')
        if not text:
            return

        center = self.world_to_screen(RoutePoint(pose.position.x, pose.position.y))
        height_px = max(14.0, getattr(marker.scale, 'z', 1.0) * self.px_per_meter)
        font = QFont(painter.font())
        font.setBold(True)
        font.setPointSizeF(max(10.0, height_px * 0.75))
        painter.save()
        painter.setFont(font)
        painter.setPen(QPen(color, 2))
        rect = QRectF(center.x() - height_px, center.y() - height_px, height_px * 2, height_px * 2)
        painter.drawText(rect, Qt.AlignCenter, text)
        painter.restore()

    def _vehicle_local_to_screen(self, vehicle_x, vehicle_y, vehicle_yaw, local_x, local_y):
        cos_yaw = math.cos(vehicle_yaw)
        sin_yaw = math.sin(vehicle_yaw)
        world_x = vehicle_x + local_x * cos_yaw - local_y * sin_yaw
        world_y = vehicle_y + local_x * sin_yaw + local_y * cos_yaw
        return self.world_to_screen(RoutePoint(world_x, world_y))

    def _vehicle_rect_to_screen_polygon(
        self, vehicle_x, vehicle_y, vehicle_yaw, local_x, local_y, length, width, local_yaw
    ):
        points = []
        for px, py in (
            (length * 0.5, width * 0.5),
            (length * 0.5, -width * 0.5),
            (-length * 0.5, -width * 0.5),
            (-length * 0.5, width * 0.5),
        ):
            rotated_x = local_x + px * math.cos(local_yaw) - py * math.sin(local_yaw)
            rotated_y = local_y + px * math.sin(local_yaw) + py * math.cos(local_yaw)
            points.append(
                self._vehicle_local_to_screen(
                    vehicle_x, vehicle_y, vehicle_yaw, rotated_x, rotated_y
                )
            )
        return QPolygonF(points)

    def _world_rect_to_screen_polygon(self, center_x, center_y, length, width, yaw):
        points = []
        for px, py in (
            (length * 0.5, width * 0.5),
            (length * 0.5, -width * 0.5),
            (-length * 0.5, -width * 0.5),
            (-length * 0.5, width * 0.5),
        ):
            world_x = center_x + px * math.cos(yaw) - py * math.sin(yaw)
            world_y = center_y + px * math.sin(yaw) + py * math.cos(yaw)
            points.append(self.world_to_screen(RoutePoint(world_x, world_y)))
        return QPolygonF(points)

    @staticmethod
    def _marker_color(marker):
        color_msg = getattr(marker, 'color', None)
        if color_msg is None:
            return QColor(37, 99, 235, 180)
        alpha = int(max(0.0, min(1.0, getattr(color_msg, 'a', 1.0))) * 180)
        return QColor(
            int(max(0.0, min(1.0, getattr(color_msg, 'r', 0.15))) * 255),
            int(max(0.0, min(1.0, getattr(color_msg, 'g', 0.39))) * 255),
            int(max(0.0, min(1.0, getattr(color_msg, 'b', 0.92))) * 255),
            alpha,
        )

    @staticmethod
    def _yaw_from_orientation(orientation):
        x = getattr(orientation, 'x', 0.0)
        y = getattr(orientation, 'y', 0.0)
        z = getattr(orientation, 'z', 0.0)
        w = getattr(orientation, 'w', 1.0)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
