import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge


class DepthObstacleDetector(Node):
    def __init__(self):
        super().__init__('depth_obstacle_detector')
        self.bridge = CvBridge()

        # 订阅深度图像
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_cb, 10)
        # 订阅相机内参
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_cb, 10)
        # 订阅激光
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)

        # 发布增强后的激光数据
        self.scan_pub = self.create_publisher(LaserScan, '/scan_enhanced', 10)

        self.camera_matrix = None
        self.latest_scan = None

        # 相机外参: 相机在base_link坐标系中的位置
        self.cam_x = 0.29   # 前向
        self.cam_y = 0.0    # 横向
        self.cam_z = 0.14   # 高度
        self.cam_pitch = 0.0  # 俯仰角(rad)
        self.get_logger().info('深度相机障碍检测启动')

    def info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def depth_cb(self, msg):
        if self.camera_matrix is None:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except:
            return

        # 1. 降采样(提高速度)
        depth_small = cv2.resize(depth, (160, 120))

        # 2. 提取3D障碍点
        obstacles = self._depth_to_obstacles(depth_small)

        # 3. 地面滤除(高度<0.05m的点去掉)
        obstacles = [p for p in obstacles if p[2] > 0.05]

        if obstacles:
            # 找到最近障碍
            closest = min(obstacles, key=lambda p: np.hypot(p[0], p[1]))
            dist = np.hypot(closest[0], closest[1])
            angle = np.degrees(np.arctan2(closest[1], closest[0]))

            # 如果是激光盲区(高于激光平面的障碍), 打印警告
            if closest[2] > 0.3:  # 高于30cm = 激光可能扫不到
                self.get_logger().info(
                    f'[深度] 悬空障碍: 距离{dist:.2f}m 角度{angle:.0f}° 高度{closest[2]:.2f}m')

    def _depth_to_obstacles(self, depth):
        h, w = depth.shape
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        points = []
        for v in range(0, h, 4):       # 步长4, 减少计算量
            for u in range(0, w, 4):
                z = depth[v, u]
                if z < 0.1 or z > 10.0 or np.isnan(z):
                    continue

                # 像素 → 相机坐标系
                x_cam = (u * w/160 - cx) * z / fx
                y_cam = (v * h/120 - cy) * z / fy
                z_cam = z

                # 相机坐标系 → base_link坐标系(简化: 忽略旋转)
                x_base = x_cam + self.cam_x
                y_base = y_cam + self.cam_y
                z_base = z_cam + self.cam_z

                points.append((x_base, y_base, z_base))

        return points


def main():
    rclpy.init()
    node = DepthObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
