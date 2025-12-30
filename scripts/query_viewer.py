#!/usr/bin/env python3
"""
CloudGuessr Query Viewer - Open3D로 query point cloud 표시

/cloudguessr/query 토픽을 구독하여 별도 창에 query를 시각화합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import json
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
        self.lock = threading.Lock()

        # Subscribers
        self.query_sub = self.create_subscription(
            PointCloud2,
            '/cloudguessr/query',
            self.on_query,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            '/cloudguessr/status',
            self.on_status,
            10
        )

        self.current_round = 0
        self.current_difficulty = ""

        # Visualizer 스레드 시작
        self.vis_thread = threading.Thread(target=self.run_visualizer, daemon=True)
        self.vis_thread.start()

        self.get_logger().info('Query Viewer 준비 완료 - /cloudguessr/query 대기 중')

    def on_query(self, msg: PointCloud2):
        """Query point cloud 수신"""
        try:
            # PointCloud2 -> numpy
            points = []
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])

            if len(points) == 0:
                return

            points = np.array(points)

            with self.lock:
                self.pcd.points = o3d.utility.Vector3dVector(points)
                # 색상 설정 (파란색 계열)
                colors = np.zeros((len(points), 3))
                colors[:, 2] = 0.8  # Blue
                colors[:, 1] = 0.5  # Some green
                self.pcd.colors = o3d.utility.Vector3dVector(colors)
                self.new_cloud = True

            self.get_logger().info(f'Query 수신: {len(points)} points')

        except Exception as e:
            self.get_logger().error(f'Query 처리 오류: {e}')

    def on_status(self, msg: String):
        """게임 상태 수신"""
        try:
            status = json.loads(msg.data)
            round_id = status.get('round_id', 0)
            difficulty = status.get('difficulty', '')

            if round_id != self.current_round:
                self.current_round = round_id
                self.current_difficulty = difficulty
                # 창 제목은 Open3D에서 동적 변경이 어려움

        except Exception as e:
            pass

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
                    self.vis.reset_view_point(True)
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
