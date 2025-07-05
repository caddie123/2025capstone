import rclpy
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt

class DepthMapNode(Node):
    def __init__(self):
        super().__init__('depth_map_node')

        # Parameters
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('resolution', 0.05)   # meters per cell
        self.declare_parameter('width', 200)         # grid width in cells
        self.declare_parameter('height', 200)        # grid height in cells
        self.declare_parameter('z_min', 0.05)        # min depth [m]
        self.declare_parameter('z_max', 10.0)        # max depth [m]

        self.frame_id   = self.get_parameter('frame_id').value
        self.resolution = self.get_parameter('resolution').value
        self.width      = self.get_parameter('width').value
        self.height     = self.get_parameter('height').value
        self.z_min      = self.get_parameter('z_min').value
        self.z_max      = self.get_parameter('z_max').value

        # Map origin centered
        self.origin_x = -(self.width  * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        # CV Bridge
        self.bridge = CvBridge()

        # Intrinsics
        self.fx = None; self.fy = None
        self.cx = None; self.cy = None

        # 1) CameraInfo 구독
        self.create_subscription(
            CameraInfo,
            '/k4a/depth/camera_info',
            self.camera_info_cb,
            10)

        # 2) Depth 이미지 구독 (ImageTransport 대신 일반 구독)
        self.create_subscription(
            Image,
            '/k4a/depth/image_raw',   # 또는 '/k4a/depth_to_rgb/image_raw'
            self.depth_cb,
            10)

        # OccupancyGrid 퍼블리셔
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/depth_occupancy_grid',
            10)

        # Matplotlib 초기화
        plt.ion()
        self.fig, self.ax = plt.subplots()
        init_grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.im = self.ax.imshow(init_grid, cmap='gray', vmin=0, vmax=100, origin='lower', interpolation='nearest')
        self.ax.set_title('Occupancy Grid')
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        for spine in self.ax.spines.values():
            spine.set_visible(False)
        plt.show()

        self.get_logger().info('DepthMapNode initialized.')

    def camera_info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]; self.fy = msg.k[4]
            self.cx = msg.k[2]; self.cy = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx}, fy={self.fy}, '
                f'cx={self.cx}, cy={self.cy}')

    def depth_cb(self, msg: Image):
        # depth image (uint16 mm -> float32 m)
        #depth_cb 내부
        depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.get_logger().info(f'Depth stats → min: {depth_m.min():.3f} m, max: {depth_m.max():.3f} m')

        if self.fx is None:
            # intrinsics 未取得なら何もしない
            return

        # 1) 유효 범위 마스크
        mask = (depth_m >= self.z_min) & (depth_m <= self.z_max)
        v, u = np.nonzero(mask)
        self.get_logger().info(f'Depth pixels in range: {v.size}')

        # 2) 월드 좌표(X–Z 평면) 계산
        z = depth_m[v, u]
        x = (u - self.cx) * z / self.fx    # 카메라 오른쪽 방향
        y = z                              # 카메라 앞쪽(지면 투영 시 Y축)

        # 3) 그리드 셀 인덱스로 변환
        gx = ((x - self.origin_x) / self.resolution).astype(np.int32)
        gy = ((y - self.origin_y) / self.resolution).astype(np.int32)
        valid = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)

        # 4) OccupancyGrid용 2D 배열 초기화 & 표시
        grid = np.zeros((self.height, self.width), dtype=np.int8)
        grid[gy[valid], gx[valid]] = 100

        # 5) OccupancyGrid 메시지 작성 및 퍼블리시
        occ = OccupancyGrid()
        occ.header.stamp = self.get_clock().now().to_msg()
        occ.header.frame_id = self.frame_id
        occ.info.resolution = self.resolution
        occ.info.width      = self.width
        occ.info.height     = self.height
        occ.info.origin.position.x = self.origin_x
        occ.info.origin.position.y = self.origin_y
        occ.info.origin.orientation.w = 1.0
        occ.data = grid.flatten().tolist()
        self.pub.publish(occ)

        # 6) Matplotlib 시각화 업데이트
        self.im.set_data(grid)
        self.im.set_clim(0,100)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = DepthMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
