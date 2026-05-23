"""Route planner panel for the control tower UI."""

import math
import os

from PyQt5.QtCore import QPoint, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPolygon
from PyQt5.QtWidgets import (
    QFileDialog,
    QLabel,
    QHBoxLayout,
    QLineEdit,
    QMenu,
    QPushButton,
    QSizePolicy,
    QWidget,
)
from PyQt5.uic import loadUi

from waywiser_teleop_py.route_files import read_waywise_route_xml
from waywiser_teleop_py.route_map import RouteMapCanvas


ACTIVE_BUTTON_STYLE = (
    'QPushButton:checked {'
    ' background-color: #2563eb;'
    ' color: white;'
    ' border: 1px solid #1d4ed8;'
    '}'
)

OPENSTREETMAP_TILE_SERVER_URL = 'http://c.osm.rrze.fau.de/osmhd'
OPENSTREETMAP_CACHE_DIR = os.path.join(
    os.environ.get(
        'WAYWISER_WS',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..')),
    ),
    'resources',
    'control_tower',
    'osm',
)

POPUP_MENU_STYLE = (
    'QMenu {'
    ' background-color: #2563eb;'
    ' color: white;'
    ' border: 1px solid #1d4ed8;'
    '}'
    'QMenu::item {'
    ' padding: 5px 18px;'
    '}'
    'QMenu::item:selected {'
    ' background-color: #1d4ed8;'
    '}'
    'QMenu::indicator:checked {'
    ' background-color: white;'
    '}'
)


class UpMenuButton(QPushButton):
    """Button with a small top-right popup arrow."""

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setBrush(QColor('#ffffff'))
        painter.setPen(Qt.NoPen)
        x = self.width() - 12
        y = 5
        painter.drawPolygon(
            QPolygon(
                [
                    QPoint(x, y + 5),
                    QPoint(x + 4, y),
                    QPoint(x + 8, y + 5),
                ]
            )
        )


class RoutePlannerWidget(QWidget):
    """Composite widget containing the map canvas and route settings."""

    send_route_requested = pyqtSignal(list, float, float)
    osm_refresh_requested = pyqtSignal()

    def __init__(self, ui_base_path, parent=None):
        super().__init__(parent)
        loadUi(os.path.join(ui_base_path, 'route_planner.ui'), self)

        self.map_canvas = RouteMapCanvas(self)
        self.map_layout.addWidget(self.map_canvas)
        self.map_canvas.route_changed.connect(self.update_route_status)
        self.clear_route_button.clicked.connect(self.map_canvas.clear_route)

        self._setup_tools_menu()
        self.load_route_button.setText('Load')
        self.clear_route_button.setText('Clear')
        self.save_route_button.setText('Save')
        self.send_route_button.setText('Send')
        self._setup_route_value_row()

        # Connect bottom controls buttons in plan mode
        self.load_route_button.clicked.connect(self.load_route)
        self.send_route_button.clicked.connect(self.request_send_route)
        self.save_route_button.clicked.connect(self.save_route)
        self.apply_speed_button.clicked.connect(self.apply_route_values)
        self.speed_spin_box.valueChanged.connect(self.update_default_route_point_values)
        self.height_spin_box.valueChanged.connect(self.update_default_route_point_values)

        self._is_quadcopter = False
        self._vehicle_connected = False
        self.map_config_widget = None
        self.update_route_status()
        self.set_vehicle_type('generic')

        # Connect hover coordinates signal and initialize label text, aligning to left
        self.map_canvas.hover_changed.connect(self.update_hover_coordinates)
        self.map_canvas.zoom_changed.connect(self.update_zoom_status)
        self.hover_coordinates = None
        self.current_zoom = self.map_canvas._osm_zoom_level()
        self._update_planning_status_label()
        self.planning_status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._setup_map_config_row()
        self._setup_osm_config_controls()
        self.route_status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.map_status_layout.addWidget(
            self.route_status_label, 0, Qt.AlignRight | Qt.AlignVCenter
        )
        self.map_status_layout.setStretch(0, 1)
        self.update_default_route_point_values()

        # Prevent the controls frame from stretching vertically and make it compact
        self.route_controls_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.route_planner_layout.setStretch(0, 1)  # Map frame gets maximum stretch
        self.route_planner_layout.setStretch(1, 0)  # Route controls frame gets 0 stretch
        for index in range(self.buttons_layout.count()):
            self.buttons_layout.setStretch(index, 1)

        self._update_controls_visibility()

    def _setup_route_value_row(self):
        self.apply_speed_button.setText('Apply')
        self.apply_speed_button.setFixedWidth(72)
        self.apply_height_button.hide()
        self.apply_height_button.setVisible(False)
        self.route_controls_layout.removeWidget(self.apply_speed_button)
        self.route_controls_layout.removeWidget(self.apply_height_button)
        self.route_controls_layout.removeWidget(self.route_status_label)
        self.route_controls_layout.removeWidget(self.height_label)
        self.route_controls_layout.removeWidget(self.height_spin_box)
        self.route_controls_layout.removeWidget(self.speed_label)
        self.route_controls_layout.removeWidget(self.speed_spin_box)
        self.route_controls_layout.addWidget(self.speed_label, 0, 0)
        self.route_controls_layout.addWidget(self.speed_spin_box, 0, 1)
        self.route_controls_layout.addWidget(self.height_label, 0, 3)
        self.route_controls_layout.addWidget(self.height_spin_box, 0, 4)
        self.route_controls_layout.addWidget(self.apply_speed_button, 0, 6)
        self.speed_spin_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.height_spin_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.route_controls_layout.setColumnStretch(0, 0)
        self.route_controls_layout.setColumnStretch(1, 1)
        self.route_controls_layout.setColumnStretch(2, 0)
        self.route_controls_layout.setColumnStretch(3, 0)
        self.route_controls_layout.setColumnStretch(4, 1)
        self.route_controls_layout.setColumnStretch(5, 0)
        self.route_controls_layout.setColumnStretch(6, 0)

    def _setup_tools_menu(self):
        self.tools_button = UpMenuButton('Tools')
        self.tools_button.setCheckable(True)
        self.tools_button.setToolTip('Route generation tools')
        self.tools_menu = QMenu(self.tools_button)
        self.tools_menu.setStyleSheet(POPUP_MENU_STYLE)
        self.zigzag_action = self.tools_menu.addAction('Zig-Zag')
        self.zigzag_action.setCheckable(True)
        self.zigzag_action.toggled.connect(self.set_zigzag_tool_enabled)
        self.tools_button.clicked.connect(self.show_tools_menu)
        self.tools_button.setStyleSheet(ACTIVE_BUTTON_STYLE)
        self.buttons_layout.insertWidget(0, self.tools_button)

    def _setup_map_config_row(self):
        status_item = self.route_controls_layout.itemAtPosition(2, 0)
        if status_item is not None:
            self.route_controls_layout.removeItem(status_item)
        self.map_config_layout = QHBoxLayout()
        self.map_config_layout.setContentsMargins(0, 0, 0, 0)
        self.route_controls_layout.addLayout(self.map_config_layout, 2, 0, 1, 7)
        self.route_controls_layout.addLayout(self.map_status_layout, 3, 0, 1, 7)

    def _setup_osm_config_controls(self):
        self.osm_url_label = QLabel('OSM URL')
        self.osm_url_edit = QLineEdit()
        self.osm_url_edit.setMinimumWidth(260)
        self.osm_url_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.osm_server_status_label = QLabel('Ready')
        self.osm_server_status_label.setFixedWidth(48)
        self.osm_server_status_label.setAlignment(Qt.AlignCenter)
        self.osm_cache_label = QLabel('Cache dir')
        self.osm_cache_edit = QLineEdit()
        self.osm_cache_label.hide()
        self.osm_cache_edit.hide()
        self.osm_cache_browse_button = QPushButton('Cache Dir')
        self.osm_cache_browse_button.setFixedWidth(104)
        self.osm_cache_browse_button.setToolTip('Select OSM tile cache directory')
        self.osm_refresh_button = QPushButton('Refresh')
        self.osm_refresh_button.setFixedWidth(96)
        self.osm_refresh_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.osm_refresh_button.setToolTip('Refresh local OSM tiles')
        self.osm_refresh_button.hide()
        self.osm_refresh_button.clicked.connect(self.osm_refresh_requested.emit)
        self.map_config_layout.addWidget(self.osm_refresh_button)
        self.map_config_layout.addWidget(self.osm_url_label)
        self.map_config_layout.addWidget(self.osm_url_edit, 1)
        self.map_config_layout.addWidget(self.osm_server_status_label)
        self.map_config_layout.addWidget(self.osm_cache_browse_button)
        self.osm_url_edit.editingFinished.connect(self.apply_osm_config_edits)
        self.osm_cache_edit.editingFinished.connect(self.apply_osm_config_edits)
        self.osm_cache_browse_button.clicked.connect(self.browse_osm_cache_dir)

    def set_osm_config_mode(self, map_source):
        is_openstreetmap = map_source == 'OpenStreetMap'
        is_local_osm = map_source == 'Local OSM server'
        show_osm_controls = is_openstreetmap or is_local_osm
        self.osm_url_label.setVisible(show_osm_controls)
        self.osm_url_edit.setVisible(show_osm_controls)
        self.osm_server_status_label.setVisible(is_local_osm)
        self.osm_url_edit.setReadOnly(is_openstreetmap)
        self.osm_cache_browse_button.setVisible(is_local_osm)
        self.osm_refresh_button.setVisible(is_local_osm)
        self.osm_cache_label.setVisible(False)
        self.osm_cache_edit.setVisible(False)
        if not is_local_osm:
            self.set_osm_server_status('Ready')

    def set_osm_server_status(self, status):
        status = 'Busy' if str(status).lower() == 'busy' else 'Ready'
        self.osm_server_status_label.setText(status)
        if status == 'Busy':
            self.osm_server_status_label.setStyleSheet('color: #b45309; font-weight: 600;')
        else:
            self.osm_server_status_label.setStyleSheet('color: #047857; font-weight: 600;')

    def show_tools_menu(self):
        menu_size = self.tools_menu.sizeHint()
        popup_pos = self.tools_button.mapToGlobal(QPoint(0, -menu_size.height()))
        self.tools_menu.exec_(popup_pos)
        self.tools_button.setChecked(self.map_canvas.active_tool is not None)

    def set_zigzag_tool_enabled(self, enabled):
        self.map_canvas.set_active_tool('zigzag' if enabled else None)
        self.tools_button.setChecked(bool(enabled))
        self._update_controls_visibility()

    def set_planning_enabled(self, enabled):
        self.map_canvas.set_planning_enabled(enabled)
        if not enabled and hasattr(self, 'zigzag_action'):
            self.zigzag_action.blockSignals(True)
            self.zigzag_action.setChecked(False)
            self.zigzag_action.blockSignals(False)
            self.tools_button.setChecked(False)
        self._update_controls_visibility()

    def set_vehicle_connected(self, connected):
        """Enable sending routes only when a vehicle node is connected."""
        self._vehicle_connected = bool(connected)
        self._update_controls_visibility()

    def update_hover_coordinates(self, x, y):
        """Update coordinates label with dynamic cursor position and map zoom level."""
        if not math.isfinite(x) or not math.isfinite(y):
            self.hover_coordinates = None
        else:
            self.hover_coordinates = (x, y)
        self._update_planning_status_label()

    def update_zoom_status(self, zoom):
        self.current_zoom = zoom
        self._update_planning_status_label()

    def _update_planning_status_label(self):
        status = f'Zoom={self.current_zoom}'
        if self.hover_coordinates is not None:
            x, y = self.hover_coordinates
            status += f'    x={x:+.2f} m  y={y:+.2f} m'
        self.planning_status_label.setText(status)

    def update_default_route_point_values(self):
        self.map_canvas.set_default_point_values(z=self.altitude(), speed=self.speed())

    def _update_controls_visibility(self):
        """Show route planning controls only in plan mode."""
        enabled = self.is_planning_enabled()
        self.route_status_label.setVisible(enabled)
        self.tools_button.setVisible(enabled)
        self.load_route_button.setVisible(enabled)
        self.clear_route_button.setVisible(enabled)
        self.save_route_button.setVisible(enabled)
        self.send_route_button.setVisible(enabled and self._vehicle_connected)
        self.send_route_button.setEnabled(enabled and self._vehicle_connected)
        self.send_route_button.setToolTip('')
        self.speed_label.setVisible(enabled)
        self.speed_spin_box.setVisible(enabled)
        self.apply_speed_button.setVisible(enabled)
        self.height_label.setVisible(enabled)
        self.height_spin_box.setVisible(enabled)
        self.apply_height_button.setVisible(False)

    def browse_osm_cache_dir(self):
        directory = QFileDialog.getExistingDirectory(
            self,
            'Select OSM Tile Cache Directory',
            self.osm_cache_edit.text() or os.path.expanduser('~'),
        )
        if directory:
            self.osm_cache_edit.setText(directory)
            self.apply_osm_config_edits()

    def apply_osm_config_edits(self):
        self.set_tile_server_url(self.osm_url_edit.text())
        self.set_tile_cache_dir(self.osm_cache_edit.text())
        self.map_canvas.refresh_tiles()

    def set_osm_controls_visible(self, visible):
        self.osm_url_label.setVisible(bool(visible))
        self.osm_url_edit.setVisible(bool(visible))
        self.osm_cache_browse_button.setVisible(bool(visible))
        self.osm_refresh_button.setVisible(bool(visible))
        self.osm_cache_label.setVisible(False)
        self.osm_cache_edit.setVisible(False)

    def apply_route_values(self):
        if not self.map_canvas.points:
            from PyQt5.QtWidgets import QMessageBox

            QMessageBox.warning(self, 'Apply Route Values', 'No route points to update.')
            return
        self.apply_speed_to_route(show_message=False)
        self.apply_height_to_route(show_message=False)
        from PyQt5.QtWidgets import QMessageBox

        QMessageBox.information(
            self,
            'Apply Route Values',
            f'Updated {len(self.map_canvas.points)} route points.',
        )

    def apply_speed_to_route(self, show_message=True):
        """Apply the current speed value to every point in the planned route."""
        from PyQt5.QtWidgets import QMessageBox

        points = self.map_canvas.points
        if not points:
            if show_message:
                QMessageBox.warning(self, "Apply Route speed", "No route points to update.")
            return

        route_speed = self.speed()
        for point in points:
            point.speed = route_speed
        self.map_canvas.update()
        if show_message:
            QMessageBox.information(
                self,
                "Apply Route speed",
                f"Updated {len(points)} route points to speed={route_speed:.2f} m/s.",
            )

    def apply_height_to_route(self, show_message=True):
        """Apply the current z value to every point in the planned route."""
        from PyQt5.QtWidgets import QMessageBox

        points = self.map_canvas.points
        if not points:
            if show_message:
                QMessageBox.warning(self, "Apply Route z", "No route points to update.")
            return

        route_height = self.altitude()
        for point in points:
            point.z = route_height
        self.map_canvas.update()
        if show_message:
            QMessageBox.information(
                self,
                "Apply Route z",
                f"Updated {len(points)} route points to z={route_height:.2f} m.",
            )

    def load_route_file(self, filename):
        """Load a preplanned WayWise XML route file into the current map frame."""
        try:
            points = read_waywise_route_xml(filename, self.map_canvas.enuref)
            if not points:
                raise ValueError("The selected route file has no points.")

            self.map_canvas.set_route_points(points)
            self.height_spin_box.setValue(points[0].z)
            self.speed_spin_box.setValue(points[0].speed)
            self.map_canvas.fit_route()
            return len(points)
        except Exception as e:
            raise RuntimeError(f"Failed to load route from {filename}: {str(e)}") from e

    def load_route(self):
        """Load a preplanned WayWise XML route into the current map frame."""
        from PyQt5.QtWidgets import QFileDialog, QMessageBox

        if self.route_points():
            choice = QMessageBox.warning(
                self,
                "Load Route",
                "Loading a route will clear the existing route on the map. Continue?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if choice != QMessageBox.Yes:
                return

        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Preplanned Route", "", "XML Files (*.xml)"
        )
        if not filename:
            return

        try:
            loaded_count = self.load_route_file(filename)
            QMessageBox.information(
                self, "Load Route", f"Loaded {loaded_count} route points from:\n{filename}"
            )
        except Exception as e:
            QMessageBox.critical(self, "Load Route", f"Failed to load route:\n{str(e)}")

    def save_route(self):
        """Export the current route to an XML file in WayWise format."""
        from PyQt5.QtWidgets import QFileDialog, QMessageBox
        
        points = self.route_points()
        if not points:
            QMessageBox.warning(self, "Save Route", "No route points to save!")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "Export Current Route to File", "", "XML Files (*.xml)"
        )
        if not filename:
            return

        if not filename.lower().endswith(".xml"):
            filename += ".xml"

        try:
            import xml.etree.ElementTree as ET
            from xml.dom import minidom

            root = ET.Element("routes")

            # 1. Write enuref element
            enuref_elem = ET.SubElement(root, "enuref")
            lat_elem = ET.SubElement(enuref_elem, "Latitude")
            lat_elem.text = f"{self.map_canvas.enuref[0]:.15f}"
            lon_elem = ET.SubElement(enuref_elem, "Longitude")
            lon_elem.text = f"{self.map_canvas.enuref[1]:.15f}"
            height_elem = ET.SubElement(enuref_elem, "Height")
            height_elem.text = f"{self.map_canvas.enuref[2]:.15f}"

            # 2. Write route element
            route_elem = ET.SubElement(root, "route")
            for point in points:
                point_elem = ET.SubElement(route_elem, "point")
                x_elem = ET.SubElement(point_elem, "x")
                x_elem.text = f"{point.x:.15f}"
                y_elem = ET.SubElement(point_elem, "y")
                y_elem.text = f"{point.y:.15f}"
                z_elem = ET.SubElement(point_elem, "z")
                z_elem.text = f"{getattr(point, 'z', self.altitude()):.6f}"
                speed_elem = ET.SubElement(point_elem, "speed")
                speed_elem.text = f"{getattr(point, 'speed', self.speed()):.6f}"
                attr_elem = ET.SubElement(point_elem, "attributes")
                attr_elem.text = "0"  # default attribute is 0

            # Pretty print the XML
            xml_str = ET.tostring(root, encoding="utf-8")
            parsed = minidom.parseString(xml_str)
            pretty_xml = parsed.toprettyxml(indent="    ")

            with open(filename, "w", encoding="utf-8") as f:
                f.write(pretty_xml)

            QMessageBox.information(
                self, "Save Route", f"Route successfully saved to:\n{filename}"
            )
        except Exception as e:
            QMessageBox.critical(self, "Save Route", f"Failed to save route:\n{str(e)}")

    def is_planning_enabled(self):
        return self.map_canvas.planning_enabled

    def route_points(self):
        return self.map_canvas.get_route_points()

    def clear_route(self):
        self.map_canvas.clear_route()

    def altitude(self):
        return self.height_spin_box.value()

    def speed(self):
        return self.speed_spin_box.value()

    def request_send_route(self):
        if not self._vehicle_connected:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(
                self, "Send Route", "Connect a vehicle node before sending a route."
            )
            return
        self.send_route_requested.emit(self.route_points(), self.altitude(), self.speed())

    def set_vehicle_pose(self, x, y, yaw_rad=0.0):
        self.map_canvas.set_vehicle_pose(x, y, yaw_rad)

    def set_vehicle_overlay_model(self, model):
        self.map_canvas.set_vehicle_overlay_model(model)

    def set_visual_markers(self, markers):
        self.map_canvas.set_visual_markers(markers)

    def set_tile_server_url(self, url):
        if hasattr(self, 'osm_url_edit') and self.osm_url_edit.text() != url:
            self.osm_url_edit.setText(url)
        self.map_canvas.set_tile_server_url(url)

    def set_tile_cache_dir(self, cache_dir):
        if hasattr(self, 'osm_cache_edit') and self.osm_cache_edit.text() != cache_dir:
            self.osm_cache_edit.setText(cache_dir)
        self.map_canvas.set_tile_cache_dir(cache_dir)

    def set_map_source(self, source):
        self.map_canvas.set_map_source(source)

    def refresh_tiles(self, clear_disk=False):
        self.map_canvas.refresh_tiles(clear_disk=clear_disk)

    def set_map_config_widget(self, widget):
        self.map_config_widget = widget
        self.map_config_layout.insertWidget(0, widget, 0, Qt.AlignLeft)

    def set_vehicle_type(self, waywise_object_type):
        self._is_quadcopter = waywise_object_type == 'quadcopter'
        self._update_controls_visibility()

    def set_enu_ref(self, enuref):
        if len(enuref) >= 3:
            self.map_canvas.set_enu_ref(enuref)

    def update_route_status(self):
        points = self.route_points()
        distance = 0.0
        for start, end in zip(points[:-1], points[1:]):
            distance += math.hypot(end.x - start.x, end.y - start.y)
        self.route_status_label.setText(f'Route: {len(points)} points, {distance:.1f} m')
