import re, json, urllib.request
import cv2, numpy as np

try:
    import pyttsx3
except ImportError:
    pyttsx3 = None

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


class AIVision(Node):

    def __init__(self, use_llm=True, llm_host='http://127.0.0.1:8001'):
        super().__init__('ai_vision')
        self.bridge = CvBridge()
        self.use_llm = use_llm
        self.llm_host = llm_host
        self.last_room = None
        self.last_sent = None       # 防止重复发送同一目标

        # 摄像头订阅
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.callback, 10)

        # Nav2 目标发布
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # 语音
        if pyttsx3:
            self.tts = pyttsx3.init()
            self.tts.setProperty('rate', 160)
        else:
            self.tts = None

        # 房间坐标映射
        self.room_map = {
            '301': (2.5, 3.0), '302': (5.0, 3.0), '303': (-2.0, 3.0),
            '201': (2.5, -3.0), '202': (5.0, -3.0), '203': (-2.0, -3.0),
            '101': (2.5, 1.0), '102': (5.0, 1.0), '103': (-2.0, 1.0),
        }

        # 清空旧日志
        open('/tmp/ai_vision_log.txt', 'w').close()
        self.get_logger().info(f'AI视觉启动 (OCR={HAS_TESSERACT} LLM={use_llm})')
        self._log('AI视觉就绪，等待识别门牌号...')

    def _log(self, msg):
        with open('/tmp/ai_vision_log.txt', 'a') as f:
            f.write(f'{msg}\n')

    def speak(self, text):
        self.get_logger().info(f'[语音] {text}')
        if self.tts:
            self.tts.say(text)
            self.tts.runAndWait()

    def _send_nav_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'  发送Nav2目标: ({x}, {y})')

    def _ocr(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        texts = []
        # 全图扫描
        if HAS_TESSERACT:
            text = pytesseract.image_to_string(thresh, config='--psm 6').strip()
            nums = re.findall(r'\d{3}', text)
            texts.extend(nums)

        # 轮廓检测 → 区域 OCR
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if not (0.5 < w / h < 5.0):
                continue
            roi = frame[y:y+h, x:x+w]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)

            if HAS_TESSERACT:
                t = pytesseract.image_to_string(roi, config='--psm 7 -c tessedit_char_whitelist=0123456789').strip()
                nums = re.findall(r'\d{3}', t)
                texts.extend(nums)

        return list(set(texts)), frame

    def _llm_decide(self, room_num, ocr_texts):
        prompt = ('你是配送机器人助手。摄像头识别到以下信息：\n'
                  f'  主要门牌号: {room_num}\n'
                  f'  所有识别数字: {ocr_texts}\n'
                  '\n'
                  '如果这是一个有效的导航目标，输出导航JSON：\n'
                  '  {"action":"nav", "x":坐标, "y":坐标, "desc":"简短播报"}\n'
                  '\n'
                  '如果识别结果无效/不可靠/不在已知房间中，用自然语言回复。\n'
                  '\n'
                  '已知房间坐标：\n'
                  '  301(2.5,3.0) 302(5.0,3.0) 303(-2.0,3.0)\n'
                  '  201(2.5,-3.0) 202(5.0,-3.0) 203(-2.0,-3.0)\n'
                  '  101(2.5,1.0) 102(5.0,1.0) 103(-2.0,1.0)\n')
        payload = {
            'model': 'Octopus-v2',
            'messages': [{'role': 'user', 'content': prompt}],
            'stream': False
        }
        try:
            req = urllib.request.Request(
                f'{self.llm_host}/rkllm_chat',
                data=json.dumps(payload).encode(),
                headers={'Content-Type': 'application/json'})
            resp = json.loads(urllib.request.urlopen(req, timeout=60).read())
            content = resp['choices'][-1]['message']['content']
            self.get_logger().info(f'[RKLLM] {content}')
            # 提取导航JSON
            for m in re.finditer(r'\{[^{}]*"action"\s*:\s*"nav"[^{}]*\}', content):
                try:
                    result = json.loads(m.group())
                    if result.get('x') is not None:
                        return float(result['x']), float(result['y']), result.get('desc', 'RKLLM')
                except Exception:
                    continue
            # 非导航回复
            return None, None, content.strip()
        except Exception as e:
            self.get_logger().warn(f'[RKLLM] {e}')
        return None, None, None

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        ocr_texts, frame = self._ocr(frame)
        room_num = ocr_texts[0] if ocr_texts else None

        # 显示 + 保存帧供网页查看
        cv2.putText(frame, f'Room: {room_num or "?"}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('AI Vision', frame)
        cv2.waitKey(1)
        cv2.imwrite('/tmp/ai_vision_frame.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])

        if not room_num or room_num == self.last_room:
            return
        self.last_room = room_num

        # 策略1: 正则匹配 (快速路径)
        if room_num in self.room_map:
            x, y = self.room_map[room_num]
            if (x, y) == self.last_sent:
                return
            self.last_sent = (x, y)
            self.get_logger().info(f'识别到房间: {room_num} → ({x}, {y})')
            self._log(f'识别: {room_num} → 导航 ({x}, {y})')
            self.speak(f'看到{room_num}房间，开始配送')
            self._send_nav_goal(x, y)
            return

        # 策略2: RKLLM 大模型决策
        if self.use_llm:
            self.get_logger().info(f'OCR结果: {room_num}, 所有文本: {ocr_texts} → 询问RKLLM')
            x, y, reply = self._llm_decide(room_num, ocr_texts)
            if x is not None:
                if (x, y) == self.last_sent:
                    return
                self.last_sent = (x, y)
                self._log(f'RKLLM: {room_num} → 导航 ({x}, {y})')
                self.speak(reply or '开始配送')
                self._send_nav_goal(x, y)
            elif reply:
                self._log(f'RKLLM回复: {reply}')
                self.speak(reply)


def main():
    rclpy.init()
    node = AIVision(use_llm=True)
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
