"""OpenStreetMap tile fetching and coordinate helpers for control tower maps."""

import math
import os
import shutil
import tempfile
import time

from PyQt5.QtCore import QObject, QStandardPaths, Qt, QUrl, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPixmap
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest


EARTH_RADIUS_M = 6378137.0
TILE_SIZE_PX = 256


def lon_to_tile_x(lon_deg, zoom):
    return int(math.floor((lon_deg + 180.0) / 360.0 * (1 << zoom)))


def lat_to_tile_y(lat_deg, zoom):
    lat_rad = math.radians(max(min(lat_deg, 85.05112878), -85.05112878))
    return int(
        math.floor(
            (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi)
            * 0.5
            * (1 << zoom)
        )
    )


def tile_x_to_lon(tile_x, zoom):
    return tile_x / float(1 << zoom) * 360.0 - 180.0


def tile_y_to_lat(tile_y, zoom):
    n = math.pi - 2.0 * math.pi * tile_y / float(1 << zoom)
    return math.degrees(math.atan(0.5 * (math.exp(n) - math.exp(-n))))


def enu_to_llh(enu_x, enu_y, enuref):
    lat0, lon0, height0 = _clean_enuref(enuref)
    lat = lat0 + math.degrees(enu_y / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(enu_x / (EARTH_RADIUS_M * max(math.cos(math.radians(lat0)), 1e-9)))
    return lat, lon, height0


def llh_to_enu(lat, lon, enuref):
    lat0, lon0, _ = _clean_enuref(enuref)
    x = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_RADIUS_M
    return x, y


def _clean_enuref(enuref):
    if len(enuref) < 3:
        return 0.0, 0.0, 0.0
    return float(enuref[0]), float(enuref[1]), float(enuref[2])


class OsmTileClient(QObject):
    """Small async OSM tile client with memory and disk cache."""

    tile_ready = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.tile_server_url = 'http://c.osm.rrze.fau.de/osmhd'
        self.cache_dir = self._make_cache_dir()
        self.memory_tiles = {}
        self.downloading = set()
        self.failed_until = {}
        self.max_downloading = 6
        self.network = QNetworkAccessManager(self)

    def set_tile_server_url(self, url):
        tile_server_url = str(url).rstrip('/')
        if tile_server_url == self.tile_server_url:
            return
        self.tile_server_url = tile_server_url
        self.memory_tiles.clear()
        self.refresh()

    def set_cache_dir(self, cache_dir):
        cache_dir = str(cache_dir)
        if cache_dir != self.cache_dir:
            self.cache_dir = cache_dir
            self.memory_tiles.clear()
            try:
                if self.cache_dir:
                    os.makedirs(self.cache_dir, exist_ok=True)
            except OSError:
                pass
        self.refresh()

    def refresh(self, clear_disk=False):
        self.memory_tiles.clear()
        self.downloading.clear()
        self.failed_until.clear()
        if clear_disk and self.cache_dir:
            self._clear_tile_dirs()
        self.tile_ready.emit()

    def _clear_tile_dirs(self):
        try:
            entries = os.listdir(self.cache_dir)
        except OSError:
            return
        for entry in entries:
            path = os.path.join(self.cache_dir, entry)
            if os.path.isdir(path) and entry.isdigit():
                try:
                    shutil.rmtree(path)
                except OSError:
                    pass

    def get_tile(self, zoom, tile_x, tile_y):
        key = (zoom, tile_x, tile_y)
        if key in self.memory_tiles:
            return self.memory_tiles[key]

        cached_path = self._tile_path(zoom, tile_x, tile_y)
        if cached_path and os.path.exists(cached_path):
            pixmap = QPixmap(cached_path)
            if not pixmap.isNull():
                self.memory_tiles[key] = pixmap
                return pixmap

        self._request_tile(zoom, tile_x, tile_y)
        return None

    def _request_tile(self, zoom, tile_x, tile_y):
        key = (zoom, tile_x, tile_y)
        max_tile = (1 << zoom) - 1
        if (
            not self.tile_server_url
            or key in self.downloading
            or self.failed_until.get(key, 0.0) > time.monotonic()
            or len(self.downloading) >= self.max_downloading
            or tile_x < 0
            or tile_y < 0
            or tile_x > max_tile
            or tile_y > max_tile
        ):
            return

        url = f'{self.tile_server_url}/{zoom}/{tile_x}/{tile_y}.png'
        request = QNetworkRequest(QUrl(url))
        request.setRawHeader(b'User-Agent', b'Waywiser-ControlTower/1.0')
        reply = self.network.get(request)
        self.downloading.add(key)
        reply.finished.connect(
            lambda reply=reply, key=key: self._tile_download_finished(reply, key)
        )

    def _tile_download_finished(self, reply, key):
        self.downloading.discard(key)
        if reply.error() == QNetworkReply.NoError:
            pixmap = QPixmap()
            if pixmap.loadFromData(reply.readAll()):
                self.memory_tiles[key] = pixmap
                self.failed_until.pop(key, None)
                self._save_tile(key, pixmap)
            else:
                self.failed_until[key] = time.monotonic() + 30.0
        else:
            self.failed_until[key] = time.monotonic() + 30.0
        reply.deleteLater()
        self.tile_ready.emit()

    def _save_tile(self, key, pixmap):
        path = self._tile_path(*key)
        if not path:
            return
        os.makedirs(os.path.dirname(path), exist_ok=True)
        pixmap.save(path, 'PNG')

    def _tile_path(self, zoom, tile_x, tile_y):
        if not self.cache_dir:
            return ''
        return os.path.join(self.cache_dir, str(zoom), str(tile_x), f'{tile_y}.png')

    def _make_cache_dir(self):
        cache_root = QStandardPaths.writableLocation(QStandardPaths.CacheLocation)
        if not cache_root:
            cache_root = os.path.join(tempfile.gettempdir(), 'waywiser_control_tower')
        cache_dir = os.path.join(cache_root, 'osm_tiles')
        try:
            os.makedirs(cache_dir, exist_ok=True)
            return cache_dir
        except OSError:
            return ''


def draw_tile_placeholder(painter, rect, text='OSM'):
    painter.fillRect(rect, QColor('#e5e7eb'))
    painter.setPen(QPen(QColor('#9ca3af'), 1))
    painter.drawRect(rect)
    painter.setPen(QColor('#4b5563'))
    painter.drawText(rect, Qt.AlignCenter, text)
