# 智能配送机器人

> 基于 RK3588 + RT-Thread 混合部署的四轮麦克纳姆轮自主配送机器人

## 项目简介

本项目面向医院、酒店等室内场景，设计了一款全自主配送机器人。Linux 侧（RK3588）运行激光雷达与深度相机融合的 SLAM 建图、A* 路径规划、VFH+ 动态避障、门牌 OCR 识别及本地语音交互等算法；RT-Thread 侧（STM32）负责多传感器融合定位、电机 PID 控制、姿态稳定及紧急停障。二者通过串口通信协同工作，实现全向精准移动与自主配送。

**核心指标**
- 导航定位精度 ±5cm
- 动态避障响应 ≤0.5s
- 停车晃动 ≤2cm


## 系统架构

```
                 
手机 App (Vue3 + Capacitor)
 │
 │  WebSocket :9090
 │
robot_server.py (Flask + rosbridge)
 │
 │  ROS2 话题
 │
 ┌──────────┬──────────┬──────────┐
 │          │          │          │
 感知层     决策层     控制层
 │          │          │
 订阅:      算法:      发布:
 /scan      A* 全局    /cmd_vel_cmd
 /camera    VFH+ 局部  /goal_pose
 /odom      队列调度
 /map       状态监控
                        │
                       STM32 电机 PID

```

## 文件说明

### `simple_go_hw.py` — 导航主控

核心决策模块，实现完整配送流程。

- **定位**：订阅 `/odom_fused`（IMU+轮式融合里程计）
- **地图**：订阅 `/map`，实时更新占据栅格用于全局规划
- **全局规划**：A* 搜索算法。将 SLAM 地图构建为 300×200 栅格，8 邻域扩展，Euclidean 距离启发，Ramer-Douglas-Peucker 路径简化
- **局部避障**：VFH+ 算法。将激光 360° 划分为 72 个扇区（每 5°），构建极坐标障碍直方图，平滑 + 二值化后寻找宽度 > 机器人角宽度的开口，选最接近目标方向的开口作为运动输出
- **配送队列**：贪心最近邻排序，语音或 App 连续下单后自动排最优配送顺序，逐个导航并播报进度
- **控制输出**：发布 `/cmd_vel_cmd`，经底盘安全层 `obstacle_avoidance` 过滤后到达电机

### `ai_voice.py` — 本地语音识别

- **语音转文本**：Whisper tiny 模型（~150MB），完全离线运行，无须联网
- **文本转语音**：pyttsx3 本地合成，配送状态实时播报
- **指令解析**：正则匹配提取房间号（"去302"、"送到103"）、坐标（"x2.5y1.5"）、别名（"回原点"）
- **工作流程**：5 秒录音 → Whisper 转文字 → 正则解析 → 调用导航接口 → 语音确认

### `ai_vision.py` — 门牌号码识别

- 订阅 `/camera/rgb/image_raw`（Orbbec Astra Pro RGB 相机）
- 图像预处理：灰度化 → 高斯模糊 → OTSU 二值化 → 形态学闭运算（合并数字笔画）
- 轮廓检测：筛选面积 > 300px 且宽高比 0.6~6.0 的矩形候选区域
- OCR 识别：Tesseract 引擎仅识别数字（`tessedit_char_whitelist=0123456789`）
- 识别到门牌号后查询房间坐标映射表，自动导航

### `depth_camera.py` — 深度相机障碍检测

- 订阅 `/camera/depth/image_raw`（Orbbec Astra Pro 深度图）
- 降采样（640×480 → 160×120）提高处理速度
- 像素坐标 → 相机坐标系 → 机器人基座坐标系（base_link）
- 滤除地面点（高度 < 5cm），提取 3D 障碍位置
- 补充激光雷达盲区：桌面高度、悬空物品

### `room_number_ocr.py` — 备选门牌识别

- 纯图像处理方案，不依赖 Tesseract
- 轮廓筛选 + 连通域分析，适用于背景噪声较大的场景

### `robot_server.py` — 手机 App 通信桥梁

- WebSocket 服务（端口 9090），基于 rosbridge 协议
- 转发机器人状态（位姿、速度、激光距离）到手机 App
- 接收 App 导航指令并发布到 ROS2 话题
- 支持地图数据实时推送

### `chassis_serial_driver_node.cpp` — 底盘串口驱动

- Linux 侧与 STM32 的串口通信桥
- 封装帧头 + 命令 + 数据 + CRC 校验的二进制协议
- 接收传感器数据（编码器、IMU），下发速度指令

### `start_all.sh` / `stop_all.sh` — 启停脚本

- 一键启动/停止全部节点
- 含进程检查和异常重启逻辑

### `快速启动命令.md` — 操作手册

- 常见命令速查表
- WiFi 配置、设备检测、故障排查


## 话题接口

### 订阅

| 话题 | 类型 | 频率 | 用途 |
|------|------|:--:|------|
| `/odom_fused` | nav_msgs/Odometry | 50Hz | 融合里程计定位 |
| `/odom` | nav_msgs/Odometry | 50Hz | 原始里程计（备用） |
| `/scan` | sensor_msgs/LaserScan | 10Hz | 激光雷达 360° 扫描 |
| `/map` | nav_msgs/OccupancyGrid | 1Hz | SLAM 占据栅格地图 |
| `/camera/rgb/image_raw` | sensor_msgs/Image | 30Hz | RGB 相机画面 |
| `/camera/depth/image_raw` | sensor_msgs/Image | 30Hz | 深度图 |

### 发布

| 话题 | 类型 | 频率 | 用途 |
|------|------|:--:|------|
| `/cmd_vel_cmd` | geometry_msgs/Twist | 20Hz | 速度指令 |
| `/goal_pose` | geometry_msgs/PoseStamped | 按需 | 导航目标 |


## 环境要求

| 组件 | 说明 |
|------|------|
| 主控 | RK3588 (6TOPS NPU) |
| 系统 | Ubuntu 22.04 |
| ROS2 | Foxy |
| 相机 | Orbbec Astra Pro |
| 激光 | RPLIDAR 360° |
| 底盘 | STM32 控制 4× 麦克纳姆轮 |
| 语音 | USB 麦克风 |


## 部署

### 1. 系统依赖
```bash
sudo apt install tesseract-ocr espeak python3-pyaudio gcc g++ cmake
```

### 2. Python 依赖
```bash
pip install openai-whisper pyttsx3 pyaudio numpy opencv-python pytesseract
```

### 3. Whisper 模型首次下载（~150MB，仅一次）
```bash
python3 -c "import whisper; whisper.load_model('tiny')"
```

### 4. 编译 C++ 节点
```bash
cd ~/ros2_ws && colcon build --packages-select chassis_controller
```

### 5. 启动
```bash
bash start_all.sh
```


## 使用流程

1. 上电启动，等待各节点就绪
2. 打开 App 或网页，确认连接状态绿灯
3. **语音操纵**："去 302 房间" → 机器人自动导航
4. **门牌号识别**：行驶中自动扫描门上号码，确认到达


## 关键参数

| 参数 | 数值 | 说明 |
|------|:--:|------|
| 最大线速度 | 0.3 m/s | 比赛限制 |
| 最大角速度 | 0.5 rad/s | |
| 避障安全距离 | 0.4 m | 前方触发 |
| 沿墙距离 | 0.35 m | Bug2 算法 |
| VFH+ 扇区数 | 72 | 每 5° |
| A* 栅格分辨率 | 0.05 m | 5cm 精度 |
| 底盘尺寸 | 0.55 × 0.38 × 0.15 m | |
| 轮距 | 0.46 m | 麦克纳姆轮 |
