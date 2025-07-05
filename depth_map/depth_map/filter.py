#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')

        # 1) 필터 범위 파라미터 선언
        self.declare_parameter('x_min', -4.0)    # y_min
        self.declare_parameter('x_max',  4.0)    # y_max
        self.declare_parameter('y_min', -1.0)    # z_max
        self.declare_parameter('y_max',  0.5)    # z_min
        self.declare_parameter('z_min',  0.5)    # x_min
        self.declare_parameter('z_max',  5.0)    # x_max

        # 2) 파라미터 값 읽기
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        self.get_logger().info(
            f'Filter ranges → x:[{self.x_min},{self.x_max}], '
            f'y:[{self.y_min},{self.y_max}], '
            f'z:[{self.z_min},{self.z_max}]'
        )

        # 퍼블리셔: 필터된 클라우드
        self.pub = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            10)

        # 구독: 원본 클라우드
        self.sub = self.create_subscription(
            PointCloud2,
            '/k4a/points2',
            self.cb_cloud,
            10)

    def cb_cloud(self, msg: PointCloud2):
        # 3) 원본 PointCloud2 를 (x,y,z) 리스트로 읽기
        points = list(pc2.read_points(
            msg,
            field_names=('x','y','z'),
            skip_nans=True
        ))

        # 4) 각 축별로 범위 필터링
        filtered = [
            (x, y, z)
            for x, y, z in points
            if (self.x_min <= x <= self.x_max)
            and (self.y_min <= y <= self.y_max)
            and (self.z_min <= z <= self.z_max)
        ]

        if not filtered:
            return

        # 5) 헤더 그대로 복사해서 새 메시지 생성
        filtered_msg = pc2.create_cloud_xyz32(
            msg.header,
            filtered
        )

        # 6) 퍼블리시
        self.pub.publish(filtered_msg)
        self.get_logger().debug(f'Published {len(filtered)} points out of {len(points)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
