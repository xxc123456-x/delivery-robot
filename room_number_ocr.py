import cv2
import numpy as np
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

try:
    import pytesseract
    HAS_TESSERACT = True
except ImportError:
    HAS_TESSERACT = False
    print("[WARN] pytesseract未安装, 回退到简单模板匹配")
    print("  安装: pip install pytesseract && sudo apt install tesseract-ocr")


class RoomNumberDetector(Node):
    def __init__(self):
        super().__init__('room_number_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.callback, 10)
        self.last_room = None

        # Nav2 目标发布
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # 房间号→坐标映射
        self.room_map = {
            '301': (2.5, 3.0),  '302': (5.0, 3.0),  '303': (-2.0, 3.0),
            '201': (2.5, -3.0), '202': (5.0, -3.0), '203': (-2.0, -3.0),
            '101': (2.5, 1.0),  '102': (5.0, 1.0),  '103': (-2.0, 1.0),
        }

        self.get_logger().info('门牌号OCR识别启动')

    def _send_nav_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'  发送Nav2目标: ({x}, {y})')

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return

        # 1. 预处理: 灰度→二值→找轮廓
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 2. OCR识别
        room_num = None
        if HAS_TESSERACT:
            # Tesseract只识别数字
            config = '--psm 7 -c tessedit_char_whitelist=0123456789'
            text = pytesseract.image_to_string(thresh, config=config).strip()
            # 取3位数字
            nums = re.findall(r'\d{3}', text)
            if nums:
                room_num = nums[0]
        else:
            # 回退: 轮廓面积筛选+简单数字匹配
            room_num = self._simple_detect(thresh)

        if room_num and room_num != self.last_room:
            self.last_room = room_num
            self.get_logger().info(f'识别到房间: {room_num}')

            if room_num in self.room_map:
                x, y = self.room_map[room_num]
                self.get_logger().info(f'  坐标: ({x}, {y}) → 发送导航目标')
                self._send_nav_goal(x, y)
            else:
                self.get_logger().info(f'  房间{room_num}未在映射表中')

        # 显示
        cv2.putText(frame, f'Room: {room_num or "?"}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('Room Number OCR', frame)
        cv2.waitKey(1)

    def _simple_detect(self, thresh):
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # 过滤小噪点
                x, y, w, h = cv2.boundingRect(cnt)
                if 0.5 < w/h < 5.0:  # 门牌号宽高比
                    roi = thresh[y:y+h, x:x+w]
                    # 在ROI里找3位数字(形态学+连通域)
                    n_white = cv2.countNonZero(roi)
                    if n_white > 100:
                        return str(int(n_white % 1000)).zfill(3)
        return None


def main():
    rclpy.init()
    node = RoomNumberDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
