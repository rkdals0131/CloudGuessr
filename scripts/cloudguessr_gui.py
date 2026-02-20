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
        scale, offset_x, offset_y, min_x, max_y = self._base_transform()
        sx = offset_x + (x - min_x) * scale
        sy = offset_y + (max_y - y) * scale
        return sx, sy

    def _screen_to_world(self, pos: QtCore.QPointF):
        min_x, max_x, min_y, max_y = self.bounds

        cx, cy = self._viewport_center()
        sx = cx + (pos.x() - self.pan_x - cx) / self.zoom
        sy = cy + (pos.y() - self.pan_y - cy) / self.zoom

        scale, offset_x, offset_y, transform_min_x, transform_max_y = self._base_transform()
        x = transform_min_x + (sx - offset_x) / scale
        y = transform_max_y - (sy - offset_y) / scale
        x = min(max_x, max(min_x, x))
        y = min(max_y, max(min_y, y))
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
        scale, offset_x, offset_y, min_x, max_y = self._base_transform()
        xs = points[:, 0].astype(np.float64, copy=False)
        ys = points[:, 1].astype(np.float64, copy=False)
        sx = offset_x + (xs - min_x) * scale
        sy = offset_y + (max_y - ys) * scale
        return QtGui.QPolygonF([QtCore.QPointF(float(x), float(y)) for x, y in zip(sx, sy)])

    def _base_transform(self):
        min_x, max_x, min_y, max_y = self.bounds
        margin = 24.0
        avail_w = max(1.0, self.width() - 2.0 * margin)
        avail_h = max(1.0, self.height() - 2.0 * margin)
        span_x = max(1e-6, max_x - min_x)
        span_y = max(1e-6, max_y - min_y)
        scale = min(avail_w / span_x, avail_h / span_y)
        content_w = span_x * scale
        content_h = span_y * scale
        offset_x = margin + (avail_w - content_w) * 0.5
        offset_y = margin + (avail_h - content_h) * 0.5
        return scale, offset_x, offset_y, min_x, max_y

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

        self.setWindowTitle('CloudGuessr - Game Console')
        self.resize(1520, 920)
        self.setStyleSheet(
            '''
            QMainWindow {
                background-color: #0f141b;
                color: #ecf2ff;
            }
            QFrame#panelBox {
                background-color: #121a24;
                border: 1px solid #26374a;
                border-radius: 10px;
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
            QLabel#metaText {
                font-size: 12px;
                color: #dce8f6;
                padding: 0px;
            }
            QLabel#stateText {
                font-size: 12px;
                font-weight: 700;
                color: #ecf2ff;
                padding: 0px;
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

        top_bar_height = 92
        top_row = QtWidgets.QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(10)
        root.addLayout(top_row)

        box_title = QtWidgets.QFrame()
        box_title.setObjectName('panelBox')
        box_title.setFixedHeight(top_bar_height)
        box_title.setSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        title_block = QtWidgets.QVBoxLayout()
        title_block.setContentsMargins(0, 0, 0, 0)
        title_block.setSpacing(2)
        title = QtWidgets.QLabel('CloudGuessr')
        title.setObjectName('title')
        subtitle = QtWidgets.QLabel(
            '메인 게임 GUI: 맵에서 클릭해 정답 제출 | Query 3D는 Open3D 보조 창에서 확인'
        )
        subtitle.setObjectName('subtitle')
        title_block.addWidget(title)
        title_block.addWidget(subtitle)
        title_wrap = QtWidgets.QVBoxLayout(box_title)
        title_wrap.setContentsMargins(14, 10, 14, 10)
        title_wrap.setSpacing(2)
        title_wrap.addLayout(title_block)
        top_row.addWidget(box_title)

        box_status = QtWidgets.QFrame()
        box_status.setObjectName('panelBox')
        box_status.setFixedHeight(top_bar_height)
        box_status.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        status_block = QtWidgets.QHBoxLayout()
        status_block.setContentsMargins(0, 0, 0, 0)
        status_block.setSpacing(14)
        self.round_text = QtWidgets.QLabel('ROUND -/-')
        self.round_text.setObjectName('metaText')
        self.state_text = QtWidgets.QLabel('STATE IDLE')
        self.state_text.setObjectName('stateText')
        self.diff_text = QtWidgets.QLabel('DIFFICULTY -')
        self.diff_text.setObjectName('metaText')
        self.round_id_text = QtWidgets.QLabel('ROUND ID -')
        self.round_id_text.setObjectName('metaText')
        self.notes_text = QtWidgets.QLabel('NOTES -')
        self.notes_text.setObjectName('metaText')
        self.notes_text.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        status_block.addWidget(self.round_text)
        status_block.addWidget(self.state_text)
        status_block.addWidget(self.diff_text)
        status_block.addWidget(self.round_id_text)
        status_block.addWidget(self.notes_text, 1)
        status_wrap = QtWidgets.QHBoxLayout(box_status)
        status_wrap.setContentsMargins(14, 10, 14, 10)
        status_wrap.setSpacing(14)
        status_wrap.addLayout(status_block, 1)
        top_row.addWidget(box_status, 1)

        button_height = top_bar_height
        self.next_btn = QtWidgets.QPushButton('Next Round')
        self.reset_btn = QtWidgets.QPushButton('Reset Round')
        self.next_btn.setFixedHeight(button_height)
        self.reset_btn.setFixedHeight(button_height)
        self.next_btn.setMinimumWidth(140)
        self.reset_btn.setMinimumWidth(140)
        top_row.addWidget(self.next_btn)
        top_row.addWidget(self.reset_btn)

        self.canvas = MapCanvas()
        root.addWidget(self.canvas, stretch=1)

        self.canvas.map_clicked.connect(self.on_map_clicked)
        self.next_btn.clicked.connect(lambda: self.node.publish_command('next_round'))
        self.reset_btn.clicked.connect(lambda: self.node.publish_command('reset_round'))

        self.bridge.map_ready.connect(self.on_map_ready)
        self.bridge.status_ready.connect(self.on_status_ready)
        self.bridge.result_ready.connect(self.on_result_ready)
        self.bridge.aligned_ready.connect(self.on_aligned_ready)
        self.bridge.log_ready.connect(self.append_log)

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

        self.current_state = state
        self.round_text.setText(f'ROUND {round_idx}/{total_rounds}')
        self.state_text.setText(f'STATE {state}')
        self.diff_text.setText(f'DIFFICULTY {difficulty}')
        self.round_id_text.setText(f'ROUND ID {round_id}')
        note_text = notes if notes else '-'
        if len(note_text) > 60:
            note_text = note_text[:57] + '...'
        self.notes_text.setText(f'NOTES {note_text}')

        if round_idx_zero != self.current_round_idx:
            self.current_round_idx = round_idx_zero
            self.canvas.clear_round_visuals(clear_aligned=True)
            self.append_log(
                f'round {round_idx}/{total_rounds} | id={round_id} | '
                f'difficulty={difficulty} | notes={note_text}'
            )
        elif state == 'WAITING_CLICK' and prev_state != 'WAITING_CLICK':
            self.canvas.clear_round_visuals(clear_aligned=True)

        state_color = {
            'IDLE': '#9fb2c7',
            'LOADING': '#f6c15c',
            'WAITING_CLICK': '#5be690',
            'SCORING': '#6bc3ff',
            'SHOWING_RESULT': '#f59af0',
        }.get(state, '#ecf2ff')
        self.state_text.setStyleSheet(f'font-size: 12px; font-weight: 700; color: {state_color};')

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
        self.node.get_logger().info(f'[{ts}] {msg}')


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
