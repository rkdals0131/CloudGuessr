#!/usr/bin/env python3
"""
CloudGuessr Query Viewer - Open3D로 query point cloud 표시

/cloudguessr/query 토픽을 구독하여 별도 창에 query를 시각화합니다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import threading


class QueryViewer(Node):
    def __init__(self):
        super().__init__('query_viewer')

        self.get_logger().info('Query Viewer 시작')

        # Open3D visualizer (별도 스레드에서 실행)
        self.vis = None
        self.pcd = o3d.geometry.PointCloud()
        self.vis_initialized = False
        self.new_cloud = False
        self.should_reset_view = False  # 라운드 변경 시에만 True
        self.lock = threading.Lock()

        # QoS for transient_local (receive last published message)
        query_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Subscribers
        self.query_sub = self.create_subscription(
            PointCloud2,
            '/cloudguessr/query',
            self.on_query,
            query_qos
        )

        self.last_rendered_round_idx = None  # query:<round_idx> 기준

        # Ready signal publisher
        self.ready_pub = self.create_publisher(String, '/cloudguessr/viewer_ready', 10)

        # Visualizer 스레드 시작
        self.vis_thread = threading.Thread(target=self.run_visualizer, daemon=True)
        self.vis_thread.start()

        self.get_logger().info('Query Viewer 준비 완료 - /cloudguessr/query 대기 중')

        # Ready 신호 발송 (1초 후부터, query 받을 때까지 반복)
        self.ready_timer = self.create_timer(1.0, self.send_ready_signal)
        self.query_received = False

    def send_ready_signal(self):
        """Query Viewer 준비 완료 신호 발송 (query 받을 때까지 반복)"""
        if self.query_received:
            self.ready_timer.cancel()
            return
        msg = String()
        msg.data = 'ready'
        self.ready_pub.publish(msg)
        self.get_logger().info('Ready 신호 발송 - Query 대기 중...')

    def on_query(self, msg: PointCloud2):
        """Query point cloud 수신"""
        try:
            self.query_received = True  # Ready 신호 반복 중지

            msg_round_idx = self._parse_round_idx(msg.header.frame_id)

            # PointCloud2 -> numpy
            points = []
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])

            if len(points) == 0:
                return

            points = np.array(points)
            should_reset = (
                self.last_rendered_round_idx is None
                or (msg_round_idx is not None and msg_round_idx != self.last_rendered_round_idx)
            )

            with self.lock:
                self.pcd.points = o3d.utility.Vector3dVector(points)
                # 색상 설정 (파란색 계열)
                colors = np.zeros((len(points), 3))
                colors[:, 2] = 0.8  # Blue
                colors[:, 1] = 0.5  # Some green
                self.pcd.colors = o3d.utility.Vector3dVector(colors)
                self.new_cloud = True
                self.should_reset_view = should_reset

            if msg_round_idx is not None:
                self.last_rendered_round_idx = msg_round_idx
                self.get_logger().info(f'Query 수신: {len(points)} points (라운드 {msg_round_idx})')
            else:
                self.get_logger().warn(
                    f'Query 수신: {len(points)} points (라운드 정보 없음: frame_id={msg.header.frame_id!r})')

        except Exception as e:
            self.get_logger().error(f'Query 처리 오류: {e}')

    @staticmethod
    def _parse_round_idx(frame_id: str):
        if not frame_id or not frame_id.startswith('query:'):
            return None
        try:
            return int(frame_id.split(':', 1)[1])
        except (IndexError, ValueError):
            return None

    def run_visualizer(self):
        """Open3D visualizer 실행 (별도 스레드)"""
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(
            window_name='CloudGuessr - Query Viewer',
            width=800,
            height=600
        )

        # 초기 설정
        self.vis.add_geometry(self.pcd)
        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])
        opt.point_size = 2.0

        # 좌표축 추가
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0)
        self.vis.add_geometry(coord_frame)

        self.vis_initialized = True

        while True:
            with self.lock:
                if self.new_cloud:
                    self.vis.update_geometry(self.pcd)
                    if self.should_reset_view:
                        self.vis.reset_view_point(True)
                        self.should_reset_view = False
                    self.new_cloud = False

            if not self.vis.poll_events():
                break
            self.vis.update_renderer()

        self.vis.destroy_window()


def main(args=None):
    rclpy.init(args=args)
    node = QueryViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
