#!/usr/bin/env python3
"""
CloudGuessr Desktop GUI (PySide6)

- Subscribes: /cloudguessr/map, /cloudguessr/status, /cloudguessr/result,
              /cloudguessr/query, /cloudguessr/aligned_query
- Publishes:  /clicked_point, /cloudguessr/command
"""

import json
import sys
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String

from PySide6 import QtCore, QtGui, QtWidgets


class RosBridge(QtCore.QObject):
    map_ready = QtCore.Signal(object)
    status_ready = QtCore.Signal(dict)
    result_ready = QtCore.Signal(dict)
    query_ready = QtCore.Signal(int)
    aligned_ready = QtCore.Signal(object)
    log_ready = QtCore.Signal(str)


class CloudGuessrGuiNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__("cloudguessr_gui")
        self.bridge = bridge
        self.map_received_once = False

        self.clicked_pub = self.create_publisher(PointStamped, "/clicked_point", 10)
        self.command_pub = self.create_publisher(String, "/cloudguessr/command", 10)

        self.create_subscription(PointCloud2, "/cloudguessr/map", self.on_map, 10)
        self.create_subscription(String, "/cloudguessr/status", self.on_status, 10)
        self.create_subscription(String, "/cloudguessr/result", self.on_result, 10)
        query_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, "/cloudguessr/query", self.on_query, query_qos)
        self.create_subscription(PointCloud2, "/cloudguessr/aligned_query", self.on_aligned_query, query_qos)

        self.get_logger().info("CloudGuessr desktop GUI node started")

    def on_map(self, msg: PointCloud2):
        # /cloudguessr/map is periodically republished by map_server.
        # Apply only the first full map in GUI to avoid repeated heavy re-caching.
        if self.map_received_once:
            return
        map_points = self._sample_cloud(msg, max_render_points=150000)
        if map_points is None:
            return
        self.map_received_once = True
        self.bridge.map_ready.emit(map_points)

    def on_aligned_query(self, msg: PointCloud2):
        aligned_points = self._sample_cloud(msg, max_render_points=20000)
        if aligned_points is None:
            return
        self.bridge.aligned_ready.emit(aligned_points)

    def _sample_cloud(self, msg: PointCloud2, max_render_points: int):
        total_points = int(msg.width) * int(msg.height)
        if total_points <= 0:
            return None

        stride = max(1, total_points // max_render_points)
        sampled = []
        for idx, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if idx % stride == 0:
                sampled.append((float(p[0]), float(p[1]), float(p[2])))

        if not sampled:
            return None
        return np.asarray(sampled, dtype=np.float32)

    def on_status(self, msg: String):
        try:
            status = json.loads(msg.data)
            self.bridge.status_ready.emit(status)
        except json.JSONDecodeError:
            self.bridge.log_ready.emit("status JSON parse failed")

    def on_result(self, msg: String):
        try:
            result = json.loads(msg.data)
            self.bridge.result_ready.emit(result)
        except json.JSONDecodeError:
            self.bridge.log_ready.emit("result JSON parse failed")

    def on_query(self, msg: PointCloud2):
        count = int(msg.width) * int(msg.height)
        self.bridge.query_ready.emit(count)

    def publish_click(self, x: float, y: float, z: float):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.clicked_pub.publish(msg)

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)


class MapCanvas(QtWidgets.QWidget):
    map_clicked = QtCore.Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(900, 700)
        self.map_points = None
        self.aligned_points = None
        self.clicked_point = None
        self.gt_point = None
        self.bounds = None
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.is_panning = False
        self.last_pan_pos = QtCore.QPointF()
        self.map_base_poly = None
        self.aligned_base_poly = None
        self.map_poly_dirty = True
        self.aligned_poly_dirty = True
        self.overlay_data = None

    def set_map_points(self, points: np.ndarray):
        self.map_points = points
        min_x = float(np.min(points[:, 0]))
        max_x = float(np.max(points[:, 0]))
        min_y = float(np.min(points[:, 1]))
        max_y = float(np.max(points[:, 1]))

        # Keep non-zero bounds for robust coordinate transforms.
        if abs(max_x - min_x) < 1e-6:
            max_x += 1.0
        if abs(max_y - min_y) < 1e-6:
            max_y += 1.0
        next_bounds = (min_x, max_x, min_y, max_y)
        if self.bounds is None or self._bounds_changed(next_bounds):
            self.bounds = next_bounds
            self.reset_view()
        else:
            self.bounds = next_bounds
        self.map_poly_dirty = True
        self.aligned_poly_dirty = True
        self.update()

    def set_result_points(self, clicked_xyz, gt_xyz):
        if clicked_xyz and len(clicked_xyz) >= 2:
            self.clicked_point = (float(clicked_xyz[0]), float(clicked_xyz[1]))
        if gt_xyz and len(gt_xyz) >= 2:
            self.gt_point = (float(gt_xyz[0]), float(gt_xyz[1]))
        self.update()

    def set_aligned_points(self, points: np.ndarray):
        self.aligned_points = points
        self.aligned_poly_dirty = True
        self.update()

    def clear_round_visuals(self, clear_aligned: bool):
        self.clicked_point = None
        self.gt_point = None
        if clear_aligned:
            self.aligned_points = None
            self.aligned_base_poly = None
            self.aligned_poly_dirty = False
        self.clear_result_overlay()
        self.update()

    def reset_view(self):
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        if self.bounds is None:
            return

        if event.button() == QtCore.Qt.LeftButton:
            x, y = self._screen_to_world(event.position())
            self.map_clicked.emit(x, y)
            return

        if event.button() == QtCore.Qt.RightButton:
            self.is_panning = True
            self.last_pan_pos = event.position()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            event.accept()
            return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        if self.is_panning:
            delta = event.position() - self.last_pan_pos
            self.pan_x += float(delta.x())
            self.pan_y += float(delta.y())
            self.last_pan_pos = event.position()
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent):
        if event.button() == QtCore.Qt.RightButton and self.is_panning:
            self.is_panning = False
            self.setCursor(QtCore.Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event: QtGui.QWheelEvent):
        if self.bounds is None:
            return

        delta_y = event.angleDelta().y()
        if delta_y == 0:
            return

        zoom_factor = 1.15 if delta_y > 0 else (1.0 / 1.15)
        prev_zoom = self.zoom
        next_zoom = min(25.0, max(0.35, prev_zoom * zoom_factor))
        if abs(next_zoom - prev_zoom) < 1e-6:
            return

        # Keep the world point under the cursor fixed while zooming.
        cursor = event.position()
        cx, cy = self._viewport_center()
        sx0 = cx + (cursor.x() - self.pan_x - cx) / prev_zoom
        sy0 = cy + (cursor.y() - self.pan_y - cy) / prev_zoom

        self.zoom = next_zoom
        self.pan_x = float(cursor.x() - cx - (sx0 - cx) * self.zoom)
        self.pan_y = float(cursor.y() - cy - (sy0 - cy) * self.zoom)
        self.update()
        event.accept()

    def resizeEvent(self, event: QtGui.QResizeEvent):
        self.map_poly_dirty = True
        self.aligned_poly_dirty = True
        super().resizeEvent(event)

    def paintEvent(self, event):
        del event
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(20, 24, 30))

        if self.map_points is None or self.bounds is None:
            painter.setPen(QtGui.QColor(190, 200, 215))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "Waiting for /cloudguessr/map ...")
            return

        self._ensure_map_poly()
        self._ensure_aligned_poly()
        painter.save()
        self._apply_view_transform(painter)
        if self.map_base_poly is not None:
            map_pen = QtGui.QPen(QtGui.QColor(115, 130, 145))
            map_pen.setWidth(0)  # cosmetic pen for crisp zoomed rendering
            painter.setPen(map_pen)
            painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
            painter.drawPoints(self.map_base_poly)
        if self.aligned_base_poly is not None:
            aligned_pen = QtGui.QPen(QtGui.QColor(70, 230, 220))
            aligned_pen.setWidth(0)  # cosmetic pen for crisp zoomed rendering
            painter.setPen(aligned_pen)
            painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
            painter.drawPoints(self.aligned_base_poly)
        painter.restore()
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        if self.clicked_point and self.gt_point:
            c_sx, c_sy = self._world_to_screen(*self.clicked_point)
            g_sx, g_sy = self._world_to_screen(*self.gt_point)
            painter.setPen(QtGui.QPen(QtGui.QColor(250, 220, 90), 2))
            painter.drawLine(c_sx, c_sy, g_sx, g_sy)

        if self.clicked_point:
            c_sx, c_sy = self._world_to_screen(*self.clicked_point)
            painter.setBrush(QtGui.QColor(255, 80, 80))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPointF(c_sx, c_sy), 6, 6)

        if self.gt_point:
            g_sx, g_sy = self._world_to_screen(*self.gt_point)
            painter.setBrush(QtGui.QColor(80, 230, 90))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPointF(g_sx, g_sy), 6, 6)

        if self.overlay_data is not None:
            self._draw_result_overlay(painter)

        painter.setPen(QtGui.QColor(160, 180, 205))
        painter.drawText(
            QtCore.QRectF(18, 12, self.width() - 36, 28),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            'Map Canvas: 좌클릭 제출 | 우클릭 드래그 팬 | 휠 줌',
        )

    def _world_to_screen(self, x: float, y: float):
        sx, sy = self._world_to_base_screen(x, y)
        cx, cy = self._viewport_center()
        sx = cx + (sx - cx) * self.zoom + self.pan_x
        sy = cy + (sy - cy) * self.zoom + self.pan_y
        return sx, sy

    def _world_to_base_screen(self, x: float, y: float):
        min_x, max_x, min_y, max_y = self.bounds
        margin = 24.0
        w = max(1.0, self.width() - 2.0 * margin)
        h = max(1.0, self.height() - 2.0 * margin)

        nx = (x - min_x) / (max_x - min_x)
        ny = (y - min_y) / (max_y - min_y)
        sx = margin + nx * w
        sy = margin + (1.0 - ny) * h
        return sx, sy

    def _screen_to_world(self, pos: QtCore.QPointF):
        min_x, max_x, min_y, max_y = self.bounds
        margin = 24.0
        w = max(1.0, self.width() - 2.0 * margin)
        h = max(1.0, self.height() - 2.0 * margin)

        cx, cy = self._viewport_center()
        sx = cx + (pos.x() - self.pan_x - cx) / self.zoom
        sy = cy + (pos.y() - self.pan_y - cy) / self.zoom

        nx = (sx - margin) / w
        ny = 1.0 - ((sy - margin) / h)
        nx = min(1.0, max(0.0, nx))
        ny = min(1.0, max(0.0, ny))

        x = min_x + nx * (max_x - min_x)
        y = min_y + ny * (max_y - min_y)
        return x, y

    def _viewport_center(self):
        return self.width() * 0.5, self.height() * 0.5

    def _apply_view_transform(self, painter: QtGui.QPainter):
        cx, cy = self._viewport_center()
        painter.translate(cx + self.pan_x, cy + self.pan_y)
        painter.scale(self.zoom, self.zoom)
        painter.translate(-cx, -cy)

    def _ensure_map_poly(self):
        if self.map_points is None or self.bounds is None:
            self.map_base_poly = None
            self.map_poly_dirty = False
            return
        if not self.map_poly_dirty and self.map_base_poly is not None:
            return
        self.map_base_poly = self._build_base_poly(self.map_points)
        self.map_poly_dirty = False

    def _ensure_aligned_poly(self):
        if self.aligned_points is None or self.bounds is None:
            self.aligned_base_poly = None
            self.aligned_poly_dirty = False
            return
        if not self.aligned_poly_dirty and self.aligned_base_poly is not None:
            return
        self.aligned_base_poly = self._build_base_poly(self.aligned_points)
        self.aligned_poly_dirty = False

    def _build_base_poly(self, points: np.ndarray):
        min_x, max_x, min_y, max_y = self.bounds
        margin = 24.0
        w = max(1.0, self.width() - 2.0 * margin)
        h = max(1.0, self.height() - 2.0 * margin)

        xs = points[:, 0].astype(np.float64, copy=False)
        ys = points[:, 1].astype(np.float64, copy=False)
        nx = (xs - min_x) / (max_x - min_x)
        ny = (ys - min_y) / (max_y - min_y)
        sx = margin + nx * w
        sy = margin + (1.0 - ny) * h
        return QtGui.QPolygonF([QtCore.QPointF(float(x), float(y)) for x, y in zip(sx, sy)])

    def set_result_overlay(
        self,
        score: int,
        status: str,
        dist_error,
        quality_pct,
        fitness,
        rmse,
        comment: str,
    ):
        self.overlay_data = {
            'score': int(score),
            'status': str(status),
            'dist_error': dist_error,
            'quality_pct': quality_pct,
            'fitness': fitness,
            'rmse': rmse,
            'comment': str(comment or ''),
        }
        self.update()

    def clear_result_overlay(self):
        self.overlay_data = None
        self.update()

    def _draw_result_overlay(self, painter: QtGui.QPainter):
        data = self.overlay_data
        if data is None:
            return

        panel_w = min(620.0, max(320.0, self.width() - 48.0))
        panel_h = 168.0
        panel_x = 24.0
        panel_y = 52.0
        panel_rect = QtCore.QRectF(panel_x, panel_y, panel_w, panel_h)

        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setBrush(QtGui.QColor(8, 14, 22, 220))
        painter.setPen(QtGui.QPen(QtGui.QColor(58, 78, 102), 1.2))
        painter.drawRoundedRect(panel_rect, 12.0, 12.0)

        score = int(data['score'])
        status = data['status']
        if status == 'FAIL':
            score_color = QtGui.QColor(255, 140, 140)
        elif score >= 4000:
            score_color = QtGui.QColor(110, 243, 169)
        elif score >= 2000:
            score_color = QtGui.QColor(255, 213, 122)
        else:
            score_color = QtGui.QColor(255, 177, 115)

        score_font = QtGui.QFont()
        score_font.setPointSize(30)
        score_font.setBold(True)
        painter.setFont(score_font)
        painter.setPen(QtGui.QPen(score_color))
        score_text = f'SCORE {score}'
        if status == 'FAIL':
            score_text += ' (FAIL)'
        painter.drawText(
            QtCore.QRectF(panel_x + 16.0, panel_y + 10.0, panel_w - 32.0, 46.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            score_text,
        )

        detail_font = QtGui.QFont()
        detail_font.setPointSize(11)
        painter.setFont(detail_font)
        painter.setPen(QtGui.QColor(216, 229, 244))

        dist_error = data['dist_error']
        if dist_error is None:
            dist_text = '거리 오차: -'
        else:
            dist_text = f'거리 오차: {float(dist_error):.2f} m'

        quality_pct = data['quality_pct']
        fitness = data['fitness']
        rmse = data['rmse']
        if quality_pct is not None:
            quality_text = f'정합 품질: {float(quality_pct):.1f}%'
        elif fitness is not None:
            quality_text = f'정합 품질: {float(fitness) * 100.0:.1f}%'
        else:
            quality_text = '정합 품질: -'
        if rmse is not None:
            quality_text += f' (RMSE {float(rmse):.3f})'

        painter.drawText(
            QtCore.QRectF(panel_x + 18.0, panel_y + 66.0, panel_w - 36.0, 24.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            dist_text,
        )
        painter.drawText(
            QtCore.QRectF(panel_x + 18.0, panel_y + 90.0, panel_w - 36.0, 24.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            quality_text,
        )

        comment = str(data['comment'] or '').strip()
        if not comment:
            comment = '다음 라운드에 도전하세요!'
        painter.setPen(QtGui.QColor(245, 250, 255))
        painter.drawText(
            QtCore.QRectF(panel_x + 18.0, panel_y + 116.0, panel_w - 36.0, 44.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop | QtCore.Qt.TextWordWrap,
            f'멘트: {comment}',
        )
        painter.restore()

    def _bounds_changed(self, next_bounds):
        if self.bounds is None:
            return True
        return any(abs(float(a) - float(b)) > 1e-3 for a, b in zip(self.bounds, next_bounds))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: CloudGuessrGuiNode, bridge: RosBridge):
        super().__init__()
        self.node = node
        self.bridge = bridge
        self.map_points = None
        self.current_state = 'IDLE'
        self.current_round_idx = -1
        self.last_status_query_points = None

        self.setWindowTitle('CloudGuessr - Game Console')
        self.resize(1520, 920)
        self.setStyleSheet(
            '''
            QMainWindow {
                background-color: #0f141b;
                color: #ecf2ff;
            }
            QLabel#title {
                font-size: 30px;
                font-weight: 700;
                color: #ffffff;
            }
            QLabel#subtitle {
                font-size: 13px;
                color: #9fb2c7;
            }
            QFrame#card {
                background-color: #151c25;
                border: 1px solid #283748;
                border-radius: 10px;
            }
            QLabel#metricTitle {
                font-size: 11px;
                color: #9fb2c7;
            }
            QLabel#metricValue {
                font-size: 19px;
                font-weight: 600;
                color: #f4f7ff;
            }
            QLabel#scoreValue {
                font-size: 42px;
                font-weight: 700;
                color: #6df3a9;
            }
            QLabel#hint {
                font-size: 12px;
                color: #99b4d1;
            }
            QPushButton {
                background-color: #2a3c52;
                border: 1px solid #3f5976;
                border-radius: 8px;
                padding: 8px 12px;
                color: #eef6ff;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #355170;
            }
            QPushButton:disabled {
                color: #6e7d8f;
                background-color: #1d2732;
                border: 1px solid #273545;
            }
            QPlainTextEdit {
                background-color: #0f151d;
                color: #d8e5f4;
                border: 1px solid #283748;
                border-radius: 8px;
            }
            '''
        )

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        header = QtWidgets.QFrame()
        header.setObjectName('card')
        header_layout = QtWidgets.QVBoxLayout(header)
        header_layout.setContentsMargins(14, 12, 14, 12)
        header_layout.setSpacing(2)

        title = QtWidgets.QLabel('CloudGuessr')
        title.setObjectName('title')
        subtitle = QtWidgets.QLabel(
            '메인 게임 GUI: 맵에서 클릭해 정답 제출 | Query 3D는 Open3D 보조 창에서 확인'
        )
        subtitle.setObjectName('subtitle')
        header_layout.addWidget(title)
        header_layout.addWidget(subtitle)
        root.addWidget(header)

        body = QtWidgets.QHBoxLayout()
        body.setSpacing(10)
        root.addLayout(body, stretch=1)

        self.canvas = MapCanvas()
        body.addWidget(self.canvas, stretch=3)

        right = QtWidgets.QFrame()
        right.setObjectName('card')
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(12, 12, 12, 12)
        right_layout.setSpacing(10)
        body.addWidget(right, stretch=1)

        status_card = QtWidgets.QFrame()
        status_card.setObjectName('card')
        status_grid = QtWidgets.QGridLayout(status_card)
        status_grid.setContentsMargins(10, 10, 10, 10)
        status_grid.setHorizontalSpacing(8)
        status_grid.setVerticalSpacing(4)

        self.round_title = QtWidgets.QLabel('ROUND')
        self.round_title.setObjectName('metricTitle')
        self.round_label = QtWidgets.QLabel('-/-')
        self.round_label.setObjectName('metricValue')

        self.state_title = QtWidgets.QLabel('STATE')
        self.state_title.setObjectName('metricTitle')
        self.state_label = QtWidgets.QLabel('IDLE')
        self.state_label.setObjectName('metricValue')

        self.diff_title = QtWidgets.QLabel('DIFFICULTY')
        self.diff_title.setObjectName('metricTitle')
        self.diff_label = QtWidgets.QLabel('-')
        self.diff_label.setObjectName('metricValue')

        self.round_id_title = QtWidgets.QLabel('ROUND ID')
        self.round_id_title.setObjectName('metricTitle')
        self.round_id_label = QtWidgets.QLabel('-')
        self.round_id_label.setObjectName('metricValue')

        self.query_title = QtWidgets.QLabel('QUERY POINTS')
        self.query_title.setObjectName('metricTitle')
        self.query_label = QtWidgets.QLabel('-')
        self.query_label.setObjectName('metricValue')

        self.notes_title = QtWidgets.QLabel('ROUND NOTES')
        self.notes_title.setObjectName('metricTitle')
        self.notes_label = QtWidgets.QLabel('-')
        self.notes_label.setObjectName('hint')
        self.notes_label.setWordWrap(True)
        self.notes_label.setMinimumHeight(36)

        status_grid.addWidget(self.round_title, 0, 0)
        status_grid.addWidget(self.state_title, 0, 1)
        status_grid.addWidget(self.diff_title, 0, 2)
        status_grid.addWidget(self.round_label, 1, 0)
        status_grid.addWidget(self.state_label, 1, 1)
        status_grid.addWidget(self.diff_label, 1, 2)
        status_grid.addWidget(self.round_id_title, 2, 0)
        status_grid.addWidget(self.query_title, 2, 1)
        status_grid.addWidget(self.round_id_label, 3, 0)
        status_grid.addWidget(self.query_label, 3, 1)
        status_grid.addWidget(self.notes_title, 4, 0, 1, 3)
        status_grid.addWidget(self.notes_label, 5, 0, 1, 3)
        right_layout.addWidget(status_card)

        score_card = QtWidgets.QFrame()
        score_card.setObjectName('card')
        score_layout = QtWidgets.QVBoxLayout(score_card)
        score_layout.setContentsMargins(10, 10, 10, 10)
        score_layout.setSpacing(4)

        score_title = QtWidgets.QLabel('SCORE')
        score_title.setObjectName('metricTitle')
        self.score_label = QtWidgets.QLabel('0')
        self.score_label.setObjectName('scoreValue')
        self.distance_label = QtWidgets.QLabel('거리 오차: -')
        self.distance_label.setObjectName('hint')
        self.quality_label = QtWidgets.QLabel('정합 품질: -')
        self.quality_label.setObjectName('hint')
        self.elapsed_label = QtWidgets.QLabel('처리 시간: -')
        self.elapsed_label.setObjectName('hint')
        self.comment_label = QtWidgets.QLabel('멘트: 라운드를 시작하세요.')
        self.comment_label.setObjectName('hint')
        self.comment_label.setWordWrap(True)
        self.next_hint_label = QtWidgets.QLabel('결과 확인 후 Next Round 버튼으로 진행')
        self.next_hint_label.setObjectName('hint')
        score_layout.addWidget(score_title)
        score_layout.addWidget(self.score_label)
        score_layout.addWidget(self.distance_label)
        score_layout.addWidget(self.quality_label)
        score_layout.addWidget(self.elapsed_label)
        score_layout.addWidget(self.comment_label)
        score_layout.addWidget(self.next_hint_label)
        right_layout.addWidget(score_card)

        control_card = QtWidgets.QFrame()
        control_card.setObjectName('card')
        control_layout = QtWidgets.QVBoxLayout(control_card)
        control_layout.setContentsMargins(10, 10, 10, 10)
        control_layout.setSpacing(8)

        button_row = QtWidgets.QHBoxLayout()
        self.next_btn = QtWidgets.QPushButton('Next Round')
        self.reset_btn = QtWidgets.QPushButton('Reset Round')
        button_row.addWidget(self.next_btn)
        button_row.addWidget(self.reset_btn)
        self.help_label = QtWidgets.QLabel('좌클릭 제출 | 우클릭 드래그 팬 | 휠 확대/축소')
        self.help_label.setObjectName('hint')
        control_layout.addLayout(button_row)
        control_layout.addWidget(self.help_label)
        right_layout.addWidget(control_card)

        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        right_layout.addWidget(self.log_view, stretch=1)

        self.canvas.map_clicked.connect(self.on_map_clicked)
        self.next_btn.clicked.connect(lambda: self.node.publish_command('next_round'))
        self.reset_btn.clicked.connect(lambda: self.node.publish_command('reset_round'))

        self.bridge.map_ready.connect(self.on_map_ready)
        self.bridge.status_ready.connect(self.on_status_ready)
        self.bridge.result_ready.connect(self.on_result_ready)
        self.bridge.query_ready.connect(self.on_query_ready)
        self.bridge.aligned_ready.connect(self.on_aligned_ready)
        self.bridge.log_ready.connect(self.append_log)
        self._reset_score_panel()

    def _reset_score_panel(self):
        self.score_label.setText('0')
        self.score_label.setStyleSheet('color: #6df3a9; font-size: 42px; font-weight: 700;')
        self.distance_label.setText('거리 오차: -')
        self.quality_label.setText('정합 품질: -')
        self.elapsed_label.setText('처리 시간: -')
        self.comment_label.setText('멘트: 라운드를 시작하세요.')
        self.next_hint_label.setText('결과 확인 후 Next Round 버튼으로 진행')
        self.canvas.clear_result_overlay()

    @QtCore.Slot(object)
    def on_map_ready(self, points):
        self.map_points = points
        self.canvas.set_map_points(points)
        self.append_log(f'map updated: {len(points)} sampled points')

    @QtCore.Slot(dict)
    def on_status_ready(self, status):
        prev_state = self.current_state
        round_idx_zero = int(status.get('round_idx', 0))
        round_idx = round_idx_zero + 1
        total_rounds = int(status.get('total_rounds', 0))
        round_id = status.get('round_id', '-')
        state = status.get('state_str', '-')
        difficulty = status.get('difficulty', '-')
        notes = str(status.get('round_notes', '') or '').strip()
        query_points = status.get('query_points')

        self.current_state = state
        self.round_label.setText(f'{round_idx}/{total_rounds}')
        self.state_label.setText(state)
        self.diff_label.setText(difficulty)
        self.round_id_label.setText(str(round_id))
        self.notes_label.setText(notes if notes else '-')
        if query_points is not None:
            self.last_status_query_points = int(query_points)
            self.query_label.setText(str(self.last_status_query_points))
        else:
            self.last_status_query_points = None

        if round_idx_zero != self.current_round_idx:
            self.current_round_idx = round_idx_zero
            self.canvas.clear_round_visuals(clear_aligned=True)
            self._reset_score_panel()
            note_text = notes if notes else '-'
            self.append_log(
                f'round {round_idx}/{total_rounds} | id={round_id} | '
                f'difficulty={difficulty} | notes={note_text}'
            )
        elif state == 'WAITING_CLICK' and prev_state != 'WAITING_CLICK':
            self.canvas.clear_round_visuals(clear_aligned=True)
            self._reset_score_panel()

        state_color = {
            'IDLE': '#9fb2c7',
            'LOADING': '#f6c15c',
            'WAITING_CLICK': '#5be690',
            'SCORING': '#6bc3ff',
            'SHOWING_RESULT': '#f59af0',
        }.get(state, '#ecf2ff')
        self.state_label.setStyleSheet(f'color: {state_color}; font-size: 19px; font-weight: 600;')

        is_scoring = state == 'SCORING'
        self.next_btn.setDisabled(is_scoring)
        self.reset_btn.setDisabled(is_scoring)

    @QtCore.Slot(dict)
    def on_result_ready(self, result):
        score = int(result.get('score', 0))
        status = result.get('status', '-')
        dist_error = result.get('dist_error_m')
        quality_pct = result.get('quality_pct')
        fitness = result.get('fitness')
        rmse = result.get('rmse')
        elapsed_ms = result.get('elapsed_ms')
        reason = result.get('reason', '')
        comment = str(result.get('comment', '') or '').strip()

        if not comment:
            if status != 'OK':
                comment = f'실패: {reason}' if reason else '정합 실패'
            elif score >= 4000:
                comment = '훌륭합니다! 거의 정확한 위치입니다!'
            elif score >= 2500:
                comment = '좋습니다! 꽤 가까운 위치입니다.'
            elif score >= 1000:
                comment = '아쉽네요. 조금 멀었습니다.'
            else:
                comment = '많이 멀었네요. 다음 라운드에 도전하세요!'

        self.score_label.setText(str(score))
        if status == 'FAIL':
            self.score_label.setStyleSheet('color: #ff8f8f; font-size: 42px; font-weight: 700;')
        elif score >= 4000:
            self.score_label.setStyleSheet('color: #6df3a9; font-size: 42px; font-weight: 700;')
        elif score >= 2000:
            self.score_label.setStyleSheet('color: #ffd57a; font-size: 42px; font-weight: 700;')
        else:
            self.score_label.setStyleSheet('color: #ffb173; font-size: 42px; font-weight: 700;')

        if dist_error is not None:
            self.distance_label.setText(f'거리 오차: {float(dist_error):.2f} m')
        else:
            self.distance_label.setText('거리 오차: -')

        if quality_pct is not None:
            quality_text = f'정합 품질: {float(quality_pct):.1f}%'
            if rmse is not None:
                quality_text += f' (RMSE {float(rmse):.3f})'
            self.quality_label.setText(quality_text)
        elif fitness is not None:
            quality_text = f'정합 품질: {float(fitness) * 100.0:.1f}%'
            if rmse is not None:
                quality_text += f' (RMSE {float(rmse):.3f})'
            self.quality_label.setText(quality_text)
        else:
            self.quality_label.setText('정합 품질: -')

        if elapsed_ms is not None:
            self.elapsed_label.setText(f'처리 시간: {float(elapsed_ms):.0f} ms')
        else:
            self.elapsed_label.setText('처리 시간: -')

        self.comment_label.setText(f'멘트: {comment}')
        self.next_hint_label.setText('Next Round 버튼으로 다음 문제로 이동')

        clicked = result.get('clicked_xyz')
        gt = result.get('gt_xyz')
        self.canvas.set_result_points(clicked, gt)
        self.canvas.set_result_overlay(
            score=score,
            status=status,
            dist_error=dist_error,
            quality_pct=quality_pct,
            fitness=fitness,
            rmse=rmse,
            comment=comment,
        )
        self.append_log(
            f'result score={score}, status={status}, dist={dist_error}, '
            f'quality={quality_pct if quality_pct is not None else fitness}, comment={comment}'
        )

    @QtCore.Slot(int)
    def on_query_ready(self, count):
        if self.last_status_query_points is None:
            self.query_label.setText(str(count))

    @QtCore.Slot(object)
    def on_aligned_ready(self, points):
        self.canvas.set_aligned_points(points)
        self.append_log(f'aligned ICP overlay updated: {len(points)} points')

    @QtCore.Slot(float, float)
    def on_map_clicked(self, x: float, y: float):
        if self.current_state != 'WAITING_CLICK':
            self.append_log(f'ignored click while {self.current_state}')
            return

        z = 0.0
        if self.map_points is not None and len(self.map_points) > 0:
            dx = self.map_points[:, 0] - x
            dy = self.map_points[:, 1] - y
            idx = int(np.argmin(dx * dx + dy * dy))
            z = float(self.map_points[idx, 2])

        self.node.publish_click(x, y, z)
        self.append_log(f'clicked ({x:.2f}, {y:.2f}, {z:.2f}) -> published /clicked_point')

    @QtCore.Slot(str)
    def append_log(self, msg: str):
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_view.appendPlainText(f'[{ts}] {msg}')


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)

    bridge = RosBridge()
    node = CloudGuessrGuiNode(bridge)
    window = MainWindow(node, bridge)
    window.show()

    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(20)

    def cleanup():
        spin_timer.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    app.aboutToQuit.connect(cleanup)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
