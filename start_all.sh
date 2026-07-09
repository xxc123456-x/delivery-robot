#!/bin/bash
# 配送机器人一键启动（分终端窗口）
# 只启动基础服务，建图/导航/相机由手机网页控制
# 用法: bash start_all.sh

set -e

WS="$HOME/Desktop/rock_ws/ros_ws"
SCRIPTS="$HOME/Desktop/ros2_scripts"
RKLLM_DIR="$WS/ai_app/RKSDK/test_rkllm_run"

# 检测终端模拟器
if command -v gnome-terminal &>/dev/null; then
    TERM="gnome-terminal -- bash -c"
elif command -v xfce4-terminal &>/dev/null; then
    TERM="xfce4-terminal -e"
elif command -v lxterminal &>/dev/null; then
    TERM="lxterminal -e bash -c"
else
    TERM="x-terminal-emulator -e bash -c"
fi

echo "========================================="
echo "  配送机器人 - 一键启动"
echo "========================================="

# 1. RKLLM 大模型
echo "[1/2] 启动 RKLLM 大模型..."
$TERM "cd '$RKLLM_DIR' && source ../.venv/bin/activate && echo '=== RKLLM 大模型 ===' && python3 flask_server.py; exec bash" &
sleep 1

# 2. 手机遥控服务器 (相机/建图/导航由手机网页控制)
echo "[2/2] 启动手机遥控..."
$TERM "source '$WS/install/setup.bash' && echo '=== 手机遥控 ===' && python3 '$SCRIPTS/robot_server.py'; exec bash" &
sleep 2

echo ""
echo "========================================="
echo "  全部启动完成"
echo "  RKLLM 大模型 : 8001"
echo "  手机遥控     : http://<机器人IP>:5000"
echo "  相机/建图/导航 : 在手机网页上操作"
echo "  停止全部     : bash $SCRIPTS/stop_all.sh"
echo "========================================="
