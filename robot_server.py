import re, json, os, subprocess, time, threading
from pathlib import Path

import cv2, numpy as np
from flask import Flask, Response, request, jsonify, render_template_string
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist

# 路径常量
WS = os.path.expanduser('~/Desktop/rock_ws/ros_ws')
SETUP = f'{WS}/install/setup.bash'
MAP_DIR = os.path.expanduser('~/Desktop/maps')

# 进程管理
current_mode = None      # 'slam' / 'nav' / None

# HTML 页面 (手机端)
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>机器人遥控</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a2e;color:#eee;font:14px sans-serif;touch-action:manipulation;padding:6px}
h2{text-align:center;margin:4px 0;font-size:17px}
.cam{border:2px solid #0f3460;border-radius:8px;width:100%;max-height:28vh;object-fit:contain;background:#000}
.sec{border:1px solid #16213e;border-radius:8px;padding:6px 8px;margin:6px 0}
.sec-title{font-size:12px;color:#888;margin-bottom:4px}
.row{display:flex;gap:6px;flex-wrap:wrap;justify-content:center;margin:4px 0}
.btn{padding:11px 14px;border:none;border-radius:6px;font-size:14px;font-weight:bold;cursor:pointer;color:#fff;white-space:nowrap}
.btn:active{opacity:.7;transform:scale(.95)}
.btn-on{background:#27ae60}
.btn-off{background:#e67e22}
.btn-nav{background:#e94560}
.btn-save{background:#0f3460}
.btn-voice{background:#0f3460}
.btn-voice.rec{background:#c0392b;animation:pulse .5s infinite alternate}
@keyframes pulse{from{opacity:1}to{opacity:.5}}
.inp{border:none;border-radius:6px;padding:9px;font-size:14px;width:75px;text-align:center;background:#16213e;color:#fff}
.dpad{display:grid;grid-template-columns:60px 60px 60px;grid-template-rows:60px 60px 60px;gap:3px;justify-content:center}
.dpad .btn{background:#16213e;font-size:20px;padding:0;display:flex;align-items:center;justify-content:center;border-radius:10px}
.dpad .btn:active{background:#0f3460}
#msg{text-align:center;margin:3px;font-size:12px;color:#888;min-height:18px}
#status{text-align:center;font-size:11px;color:#555;margin-bottom:2px;line-height:1.5}
</style>
</head>
<body>
<h2>🤖 配送机器人</h2>
<div id="status">加载中...</div>
<div id="msg"></div>

<img class="cam" id="camImg" src="/snapshot/camera" alt="摄像头">
<img class="cam" id="mapImg" src="/snapshot/map" style="max-height:24vh;margin-top:3px" alt="地图">

<!-- 传感器 -->
<div class="sec">
<div class="sec-title">📷 传感器</div>
<div class="row">
  <button class="btn btn-on" onclick="camera_start()">启动相机</button>
  <button class="btn btn-off" onclick="camera_stop()">停止相机</button>
</div>
</div>

<!-- 建图 & 导航 -->
<div class="sec">
<div class="sec-title">🗺️ 建图 & 导航</div>
<div class="row">
  <button class="btn btn-on" onclick="slam_start()">开始建图</button>
  <button class="btn btn-off" onclick="slam_stop()">停止建图</button>
</div>
<div class="row">
  <input class="inp" id="mapname" placeholder="地图名" value="my_map" style="width:85px">
  <button class="btn btn-save" onclick="slam_save()">保存地图</button>
  <button class="btn btn-nav" onclick="nav_start()">启动导航</button>
  <button class="btn btn-off" onclick="nav_stop()">停止导航</button>
</div>
</div>

<!-- 遥控 -->
<div class="sec">
<div class="sec-title">🎮 方向遥控 <span id="chassisLight" style="color:#e74c3c">●</span></div>
<div class="dpad">
  <span></span>
  <button class="btn" onpointerdown="cmd('fwd')" onpointerup="cmd('stop')" onpointerleave="cmd('stop')">▲</button>
  <span></span>
  <button class="btn" onpointerdown="cmd('left')" onpointerup="cmd('stop')" onpointerleave="cmd('stop')">◀</button>
  <button class="btn" style="background:#c0392b;font-size:15px" onclick="cmd('stop')">STOP</button>
  <button class="btn" onpointerdown="cmd('right')" onpointerup="cmd('stop')" onpointerleave="cmd('stop')">▶</button>
  <button class="btn" onpointerdown="cmd('ccw')" onpointerup="cmd('stop')" onpointerleave="cmd('stop')">↺</button>
  <button class="btn" onpointerdown="cmd('back')" onpointerup="cmd('stop')" onpointerleave="cmd('stop')">▼</button>
  <button class="btn" onpointerdown="cmd('cw')" onpointerup="cmd('stop')" onpointerleave="cmd('stop')">↻</button>
</div>
</div>

<!-- 导航目标 -->
<div class="sec">
<div class="sec-title">🎯 导航目标</div>
<div class="row">
  <input class="inp" id="navx" placeholder="X" value="5.0">
  <input class="inp" id="navy" placeholder="Y" value="3.0">
  <button class="btn btn-nav" onclick="nav_goal()">发送目标</button>
</div>
</div>

<!-- 语音 -->
<div class="sec">
<div class="sec-title">🎤 语音识别</div>
<div class="row">
  <button class="btn btn-on" id="voiceBtn" onclick="voice_start()">启动语音</button>
  <button class="btn btn-off" onclick="voice_stop()">停止语音</button>
</div>
</div>

<!-- 视觉 -->
<div class="sec">
<div class="sec-title">👁️ 深度识别</div>
<div class="row">
  <button class="btn btn-on" onclick="vision_start()">启动识别</button>
  <button class="btn btn-off" onclick="vision_stop()">停止识别</button>
</div>
<div id="visionResult" style="font-size:12px;color:#aaa;min-height:18px;text-align:center;margin-top:4px">识别到的房间号将显示在这里</div>
</div>

<script>
function api(url,data){return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data||{})}).then(r=>r.json())}
function cmd(a){api('/api/cmd',{action:a})}
function camera_start(){document.getElementById('msg').innerText='正在启动相机...';api('/api/camera/start').then(d=>{document.getElementById('msg').innerText=d.msg})}
function camera_stop(){api('/api/camera/stop').then(d=>{document.getElementById('msg').innerText=d.msg})}
function slam_start(){document.getElementById('msg').innerText='正在启动建图...';api('/api/slam/start').then(d=>{document.getElementById('msg').innerText=d.msg})}
function slam_stop(){api('/api/slam/stop').then(d=>{document.getElementById('msg').innerText=d.msg})}
function slam_save(){var n=document.getElementById('mapname').value;document.getElementById('msg').innerText='正在保存...';api('/api/slam/save',{name:n}).then(d=>{document.getElementById('msg').innerText=d.msg})}
function nav_start(){var n=document.getElementById('mapname').value;document.getElementById('msg').innerText='正在启动导航...';api('/api/nav/start',{map_name:n}).then(d=>{document.getElementById('msg').innerText=d.msg})}
function nav_stop(){document.getElementById('msg').innerText='正在停止导航...';api('/api/nav/stop').then(d=>{document.getElementById('msg').innerText=d.msg})}
function nav_goal(){
  var x=parseFloat(document.getElementById('navx').value);
  var y=parseFloat(document.getElementById('navy').value);
  api('/api/nav',{x:x,y:y}).then(d=>{document.getElementById('msg').innerText=d.msg||''});
  document.getElementById('msg').innerText='导航目标已发送';
}
function voice_start(){
  document.getElementById('msg').innerText='正在启动语音识别...';
  api('/api/voice').then(d=>{document.getElementById('msg').innerText=d.text||d.msg})
}
function voice_stop(){
  api('/api/voice/stop').then(d=>{
    document.getElementById('msg').innerText=d.msg;

  })
}
var visionActive=false;
function vision_start(){
  document.getElementById('msg').innerText='正在启动深度识别...';
  api('/api/vision/start').then(d=>{
    document.getElementById('msg').innerText=d.msg||d.text;
    visionActive=true;
    document.getElementById('camImg').src='/snapshot/vision?t='+Date.now();
  })
}
function vision_stop(){
  api('/api/vision/stop').then(d=>{
    document.getElementById('msg').innerText=d.msg;
    document.getElementById('visionResult').innerText='已停止';
    visionActive=false;
    document.getElementById('camImg').src='/snapshot/camera?t='+Date.now();
  })
}
setInterval(()=>{fetch('/api/status').then(r=>r.json()).then(d=>{
  var s='X:'+d.x.toFixed(2)+' Y:'+d.y.toFixed(2);
  s+=' | 底盘:'+(d.chassis?'🟢':'🔴');
  s+=' RKLLM:'+(d.rkllm?'✓':'✗');
  s+=' 相机:'+(d.cam?'✓':'✗');
  s+=' 雷达:'+(d.scan?'✓':'✗');
  s+=' SLAM:'+(d.slam?'✓':'✗');
  s+=' 导航:'+(d.nav?'✓':'✗');
  document.getElementById('status').innerText=s;
  var light=document.getElementById('chassisLight');
  if(light)light.style.color=d.chassis?'#2ecc71':'#e74c3c';
})},1000);
setInterval(()=>{
  var src=visionActive?'/snapshot/vision':'/snapshot/camera';
  document.getElementById('camImg').src=src+'?t='+Date.now();
},200);
setInterval(()=>{
  document.getElementById('mapImg').src='/snapshot/map?t='+Date.now();
},500);
setInterval(()=>{
  fetch('/api/vision/result').then(r=>r.json()).then(d=>{
    if(d.lines&&d.lines.length>0){
      document.getElementById('visionResult').innerText=d.lines.slice(-3).join(' | ');
    }
  }).catch(()=>{});
},1000);
</script>
</body>
</html>"""

# ROS 节点
class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_server_node')
        self.bridge = CvBridge()
        self._frame = None
        self._frame_lock = threading.Lock()
        self._map = None
        self._map_lock = threading.Lock()
        self._scan = None
        self._scan_lock = threading.Lock()
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self._last_odom_time = 0.0
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # 摄像头
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self._img_cb, sensor_qos)
        # 里程计
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, reliable_qos)
        # 地图
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self._map_cb, reliable_qos)
        # 雷达
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._scan_cb, sensor_qos)
        # 导航
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # 控制
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._frame_lock:
                self._frame = frame
        except Exception as e:
            self.get_logger().error(f'img_cb failed: {e}', throttle_duration_sec=5)

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self._last_odom_time = time.time()

    def _map_cb(self, msg):
        with self._map_lock:
            self._map = msg

    def _scan_cb(self, msg):
        with self._scan_lock:
            self._scan = msg

    def get_frame(self):
        with self._frame_lock:
            return self._frame.copy() if self._frame is not None else None

    def get_map(self):
        with self._map_lock:
            return self._map

    def has_scan(self):
        with self._scan_lock:
            return self._scan is not None

    def chassis_online(self):
        return (time.time() - self._last_odom_time) < 1.0

    def get_map_image(self):
        with self._map_lock:
            if self._map is None:
                return None
            m = self._map
            w, h = m.info.width, m.info.height
            res = m.info.resolution
            ox, oy = m.info.origin.position.x, m.info.origin.position.y
            data = np.array(m.data, dtype=np.int8).reshape(h, w)
            # -1未知→灰, 0空闲→白, 100障碍→黑
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[data == -1] = [205, 200, 205]
            img[data == 0] = [255, 255, 255]
            img[data == 100] = [50, 50, 50]
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # 绘制机器人和激光在 OpenCV 坐标系 (y轴翻转)
        rx = int((self.x - ox) / res)
        ry = h - 1 - int((self.y - oy) / res)
        cyaw = -self.yaw  # ROS

        # 激光扫描点
        with self._scan_lock:
            if self._scan is not None:
                s = self._scan
                angle = s.angle_min
                for r in s.ranges:
                    if s.range_min < r < s.range_max:
                        lx = int((self.x + r * np.cos(self.yaw + angle) - ox) / res)
                        ly = h - 1 - int((self.y + r * np.sin(self.yaw + angle) - oy) / res)
                        if 0 <= lx < w and 0 <= ly < h:
                            img[ly, lx] = [255, 0, 0]
                    angle += s.angle_increment

        # 机器人位置
        if 0 <= rx < w and 0 <= ry < h:
            cv2.circle(img, (rx, ry), 4, (0, 255, 0), -1)
            ax = int(rx + 12 * np.cos(cyaw))
            ay = int(ry + 12 * np.sin(cyaw))
            cv2.arrowedLine(img, (rx, ry), (ax, ay), (0, 255, 0), 2, tipLength=0.4)

        return img

    def send_nav(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Nav2: ({x}, {y})')

    def send_cmd(self, action):
        t = Twist()
        v, w = 0.3, 0.8
        if action == 'fwd': t.linear.x = v
        elif action == 'back': t.linear.x = -v
        elif action == 'left': t.linear.y = v
        elif action == 'right': t.linear.y = -v
        elif action == 'ccw': t.angular.z = w
        elif action == 'cw': t.angular.z = -w
        self.cmd_pub.publish(t)


# Flask
app = Flask(__name__)
robot = None

# 房间坐标
ROOM_MAP = {
    '301': (2.5, 3.0), '302': (5.0, 3.0), '303': (-2.0, 3.0),
    '201': (2.5, -3.0), '202': (5.0, -3.0), '203': (-2.0, -3.0),
    '101': (2.5, 1.0), '102': (5.0, 1.0), '103': (-2.0, 1.0),
}
LLM_HOST = 'http://127.0.0.1:8001'


@app.route('/')
def index():
    return render_template_string(HTML_PAGE)


def _placeholder(text, w=640, h=480):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(img, text, (w//2-120, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (128,128,128), 2)
    return img

@app.route('/camera')
def camera():
    def gen():
        while True:
            frame = robot.get_frame()
            if frame is None:
                frame = _placeholder('Waiting for camera...')
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.1)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/snapshot/camera')
def snapshot_camera():
    frame = robot.get_frame()
    if frame is None:
        frame = _placeholder('Waiting for camera...')
    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


@app.route('/snapshot/vision')
def snapshot_vision():
    path = '/tmp/ai_vision_frame.jpg'
    if os.path.exists(path):
        with open(path, 'rb') as f:
            return Response(f.read(), mimetype='image/jpeg')
    frame = _placeholder('Waiting for AI Vision...')
    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


@app.route('/snapshot/map')
def snapshot_map():
    img = robot.get_map_image()
    if img is None:
        img = _placeholder('Waiting for SLAM / Nav...')
    _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


@app.route('/map')
def map_view():
    def gen():
        while True:
            img = robot.get_map_image()
            if img is None:
                img = _placeholder('Waiting for SLAM / Nav...')
            _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.5)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/status')
def api_status():
    return jsonify({
        'x': robot.x, 'y': robot.y, 'yaw': robot.yaw,
        'rkllm': _check_rkllm(),
        'chassis': robot.chassis_online(),
        'cam': robot.get_frame() is not None,
        'scan': robot.has_scan(),
        'slam': _check_proc('slam_gmapping'),
        'nav': _check_proc('rt_robot_nav2'),
        'map': robot.get_map() is not None,
    })


@app.route('/api/nav', methods=['POST'])
def api_nav():
    data = request.get_json()
    x, y = float(data['x']), float(data['y'])
    robot.send_nav(x, y)
    return jsonify({'ok': True, 'msg': f'导航: ({x:.1f}, {y:.1f})'})


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    action = request.get_json().get('action', 'stop')
    robot.send_cmd(action)
    return jsonify({'ok': True, 'action': action})


@app.route('/api/voice', methods=['POST'])
def api_voice():
    script = os.path.expanduser('~/Desktop/ros2_scripts/ai_voice.py')
    if not os.path.exists(script):
        return jsonify({'text': f'语音脚本不存在: {script}', 'nav': None})
    # ai_voice
    term = _detect_terminal()
    subprocess.Popen(f'{term} "echo \'=== AI Voice ===\' && python3 {script}; exec bash"', shell=True)
    return jsonify({'text': '语音助手已启动，请在终端窗口说话', 'nav': None})


@app.route('/api/voice/stop', methods=['POST'])
def api_voice_stop():
    subprocess.run('pkill -f ai_voice.py 2>/dev/null', shell=True)
    return jsonify({'ok': True, 'msg': '语音识别已停止'})



@app.route('/api/vision/start', methods=['POST'])
def api_vision_start():
    script = os.path.expanduser('~/Desktop/ros2_scripts/ai_vision.py')
    if not os.path.exists(script):
        return jsonify({'msg': f'视觉脚本不存在: {script}'})
    _launch_term('AI Vision', f'python3 {script}')
    return jsonify({'msg': '深度识别已启动'})


@app.route('/api/vision/stop', methods=['POST'])
def api_vision_stop():
    subprocess.run('pkill -f ai_vision.py 2>/dev/null', shell=True)
    return jsonify({'ok': True, 'msg': '深度识别已停止'})


@app.route('/api/vision/result')
def api_vision_result():
    log = '/tmp/ai_vision_log.txt'
    lines = []
    if os.path.exists(log):
        try:
            with open(log) as f:
                lines = [l.strip() for l in f.readlines() if l.strip()]
        except Exception:
            pass
    return jsonify({'lines': lines})


def parse_command(text):
    tl = text.lower().replace(' ', '')
    cn = {'零':'0','一':'1','二':'2','三':'3','四':'4','五':'5','六':'6','七':'7','八':'8','九':'9'}
    for c, d in cn.items():
        tl = tl.replace(c, d)

    # 房间号
    m = re.search(r'(\d{3})', tl)
    if m and m.group(1) in ROOM_MAP:
        r = m.group(1); return ROOM_MAP[r][0], ROOM_MAP[r][1], f'房间{r}'

    # 坐标
    m = re.search(r'x\s*([-]?\d+\.?\d*)\s*y\s*([-]?\d+\.?\d*)', tl)
    if not m: m = re.search(r'([-]?\d+\.?\d*)\s*[,，]\s*([-]?\d+\.?\d*)', tl)
    if m: return float(m.group(1)), float(m.group(2)), f'坐标({m.group(1)},{m.group(2)})'

    # 别名
    aliases = {'原点': (0,0), '充电桩': (0,0), '门口': (0,0)}
    for a, p in aliases.items():
        if a in tl: return p[0], p[1], a

    # RKLLM
    try:
        import urllib.request as ur
        prompt = ('你是配送机器人助手。输出JSON:\n{"action":"nav","x":坐标,"y":坐标,"desc":"描述"}\n'
                  '或者自然语言回复。已知: 301(2.5,3.0) 302(5.0,3.0) 原点(0,0)。\n指令:' + text)
        p = json.dumps({'model':'Octopus-v2','messages':[{'role':'user','content':prompt}],'stream':False})
        req = ur.Request(f'{LLM_HOST}/rkllm_chat', data=p.encode(), headers={'Content-Type':'application/json'})
        resp = json.loads(ur.urlopen(req, timeout=30).read())
        content = resp['choices'][-1]['message']['content']
        for m in re.finditer(r'\{[^{}]*"action"\s*:\s*"nav"[^{}]*\}', content):
            r = json.loads(m.group())
            if r.get('x') is not None:
                return float(r['x']), float(r['y']), r.get('desc','RKLLM')
        return None, None, content.strip()
    except Exception as e:
        return None, None, f'大模型: {e}'


def _check_rkllm():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        s.connect(('127.0.0.1', 8001))
        s.close()
        return True
    except Exception:
        return False


def _check_proc(name):
    p = f'[{name[0]}]{name[1:]}'
    return subprocess.run(f'pgrep -f "{p}" >/dev/null 2>&1', shell=True).returncode == 0


def _detect_terminal():
    import shutil
    if shutil.which('gnome-terminal'):
        return 'gnome-terminal -- bash -c'
    elif shutil.which('xfce4-terminal'):
        return 'xfce4-terminal -e'
    elif shutil.which('lxterminal'):
        return 'lxterminal -e bash -c'
    else:
        return 'x-terminal-emulator -e bash -c'


def _launch_term(title, cmd):
    term = _detect_terminal()
    full = f"source {SETUP} && echo '=== {title} ===' && {cmd}; exec bash"
    subprocess.Popen(f'{term} "{full}"', shell=True)


def _kill_all():
    subprocess.run('pkill -f slam_gmapping 2>/dev/null; '
                   'pkill -f rt_robot_nav2 2>/dev/null; '
                   'pkill -f chassis_serial_driver 2>/dev/null; '
                   'pkill -f rplidar_composition 2>/dev/null', shell=True)


@app.route('/api/camera/start', methods=['POST'])
def api_camera_start():
    if _check_proc('astra_camera'):
        return jsonify({'ok': True, 'msg': '摄像头已在运行'})
    _launch_term('Camera', 'ros2 launch astra_camera astra_pro.launch.xml')
    time.sleep(3)
    return jsonify({'ok': True, 'msg': '摄像头已启动'})


@app.route('/api/camera/stop', methods=['POST'])
def api_camera_stop():
    subprocess.run('pkill -f astra_camera 2>/dev/null', shell=True)
    return jsonify({'ok': True, 'msg': '摄像头已停止'})


@app.route('/api/slam/start', methods=['POST'])
def api_slam_start():
    global current_mode
    _kill_all()
    current_mode = 'slam'
    time.sleep(1)
    _launch_term('SLAM', 'ros2 launch slam_gmapping slam_gmapping.launch.py')
    return jsonify({'ok': True, 'msg': 'SLAM 建图已启动，遥控小车走场地'})


@app.route('/api/slam/stop', methods=['POST'])
def api_slam_stop():
    global current_mode
    current_mode = None
    _kill_all()
    return jsonify({'ok': True, 'msg': 'SLAM 建图已停止'})


@app.route('/api/nav/stop', methods=['POST'])
def api_nav_stop():
    global current_mode
    current_mode = None
    subprocess.run('pkill -f rt_robot_nav2 2>/dev/null', shell=True)
    return jsonify({'ok': True, 'msg': '导航已停止'})


@app.route('/api/slam/save', methods=['POST'])
def api_slam_save():
    data = request.get_json()
    map_name = data.get('name', 'my_map')
    os.makedirs(MAP_DIR, exist_ok=True)
    # 直接调用 map_saver_cli，绕过 launch 文件以避免路径和 ImageMagick/ARM64 兼容问题
    _launch_term('Save Map',
        f'ros2 run nav2_map_server map_saver_cli '
        f'-f {MAP_DIR}/{map_name} '
        f'--ros-args -p map_subscribe_transient_local:=true')
    return jsonify({'ok': True, 'msg': f'地图保存中，请查看终端窗口。文件将存到 ~/Desktop/maps/{map_name}.yaml'})


@app.route('/api/nav/start', methods=['POST'])
def api_nav_start():
    global current_mode
    data = request.get_json()
    map_name = data.get('map_name', 'my_map')
    map_file = f'{MAP_DIR}/{map_name}.yaml'
    _kill_all()
    current_mode = 'nav'
    time.sleep(1)
    _launch_term('Navigation',
        f'ros2 launch rt_robot_nav2 rt_robot_nav2_complete.launch.py '
        f'use_slam:=false use_nav:=true map_file:={map_file} open_rviz:=true')
    return jsonify({'ok': True, 'msg': f'导航已启动，地图: {map_name}'})


def main():
    global robot
    rclpy.init()
    robot = RobotNode()

    # ROS spin 线程
    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.01)
    threading.Thread(target=ros_spin, daemon=True).start()

    # Flask
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == '__main__':
    main()
