#!/usr/bin/env bash
# ROS 2问答助手一键构建和运行脚本
set -eo pipefail

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_NAME="ros2_qa_assistant"

echo "[信息] 工作空间: $WS_DIR"

# 清理之前运行的进程
echo "[信息] 清理现有的QA助手进程..."
pkill -f "qa_core_node" >/dev/null 2>&1 || true
pkill -f "web_input_node" >/dev/null 2>&1 || true
pkill -f "output_manager_node" >/dev/null 2>&1 || true
pkill -f "knowledge_base_server" >/dev/null 2>&1 || true
pkill -f "qa_assistant_launch.py" >/dev/null 2>&1 || true
pkill -f "ros2 launch.*ros2_qa_assistant" >/dev/null 2>&1 || true

if [ ! -f "/opt/ros/rolling/setup.bash" ]; then
  echo "[错误] 未找到ROS 2 Rolling，请安装ROS 2 Rolling" >&2
  exit 1
fi

echo "[信息] 加载ROS Rolling环境"
source /opt/ros/rolling/setup.bash

# 使用临时目录存放ROS日志
export ROS_LOG_DIR="/tmp/ros_logs"
mkdir -p "$ROS_LOG_DIR"

echo "[信息] 清理之前的构建文件"
cd "$WS_DIR"
if [ -d "build" ]; then
  rm -rf build
  echo "[信息] 删除build目录"
fi
if [ -d "install" ]; then
  rm -rf install
  echo "[信息] 删除install目录"
fi
if [ -d "log" ]; then
  rm -rf log
  echo "[信息] 删除log目录"
fi

echo "[信息] 构建包: $PKG_NAME"
# 检查colcon命令
if ! command -v colcon >/dev/null 2>&1; then
  echo "[错误] 找不到colcon命令" >&2
  exit 1
fi
colcon build --packages-select "$PKG_NAME"

echo "[信息] 加载工作空间环境"
source "$WS_DIR/install/setup.bash"

# 检查rosbridge是否安装
if ! command -v ros2 >/dev/null; then
  echo "[错误] 加载环境后找不到ros2命令" >&2
  exit 1
fi

set +e
ros2 pkg prefix rosbridge_server >/dev/null 2>&1
HAS_ROSBRIDGE=$?
set -e

ROSBRIDGE_PID=""
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
if [ "$HAS_ROSBRIDGE" -ne 0 ]; then
  echo "[警告] 未检测到rosbridge_server，跳过启动。安装命令: sudo apt install ros-rolling-rosbridge-server" >&2
else
  echo "[信息] 在端口${ROSBRIDGE_PORT}后台启动rosbridge_websocket"
  ROSBRIDGE_PARAMS_FILE="$WS_DIR/src/$PKG_NAME/config/rosbridge_qos.yaml"
  if [ -f "$ROSBRIDGE_PARAMS_FILE" ]; then
    echo "[信息] 使用rosbridge参数文件: $ROSBRIDGE_PARAMS_FILE"
    ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=$ROSBRIDGE_PORT --params-file "$ROSBRIDGE_PARAMS_FILE" >/tmp/rosbridge.log 2>&1 &
  else
    echo "[警告] 未找到rosbridge_qos.yaml，不使用参数文件启动"
    ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=$ROSBRIDGE_PORT >/tmp/rosbridge.log 2>&1 &
  fi
  ROSBRIDGE_PID=$!
  sleep 1
  # 等待rosbridge节点就绪
  echo "[信息] 等待rosbridge节点就绪..."
  for i in $(seq 1 20); do
    if ros2 node list 2>/dev/null | grep -q rosbridge; then
      echo "[信息] rosbridge已就绪"
      break
    fi
    sleep 0.5
  done
  if ! ros2 node list 2>/dev/null | grep -q rosbridge; then
    echo "[警告] rosbridge未在节点列表中出现，请检查/tmp/rosbridge.log和端口${ROSBRIDGE_PORT}"
    echo "[信息] 如果端口被占用，可设置不同端口: export ROSBRIDGE_PORT=9091"
  fi
fi

cleanup() {
  echo "\n[信息] 正在关闭..."
  
  # 杀死相关的ROS2节点
  echo "[信息] 停止QA助手节点..."
  pkill -f "qa_core_node" >/dev/null 2>&1 || true
  pkill -f "web_input_node" >/dev/null 2>&1 || true
  pkill -f "output_manager_node" >/dev/null 2>&1 || true
  pkill -f "knowledge_base_server" >/dev/null 2>&1 || true
  pkill -f "qa_assistant_launch.py" >/dev/null 2>&1 || true
  
  # 杀死所有ros2 launch进程
  echo "[信息] 停止所有ros2 launch进程..."
  pkill -f "ros2 launch" >/dev/null 2>&1 || true
  
  # 杀死rosbridge进程
  if [ -n "$ROSBRIDGE_PID" ] && ps -p "$ROSBRIDGE_PID" >/dev/null 2>&1; then
    echo "[信息] 杀死rosbridge进程 ($ROSBRIDGE_PID)"
    kill "$ROSBRIDGE_PID" >/dev/null 2>&1 || true
  fi
  
  # 清理剩余的ROS2进程
  echo "[信息] 清理剩余的ROS2进程..."
  pkill -f "_ros2_daemon" >/dev/null 2>&1 || true
  
  echo "[信息] 清理完成"
}
trap cleanup EXIT INT TERM

echo "[信息] 启动QA助手"
WEB_FILE="file://$WS_DIR/src/$PKG_NAME/web/index.html"
echo "[信息] Web页面: $WEB_FILE"

# 尝试自动打开网页
if command -v xdg-open >/dev/null 2>&1; then
  (xdg-open "$WEB_FILE" >/dev/null 2>&1 || true) &
elif command -v gio >/dev/null 2>&1; then
  (gio open "$WEB_FILE" >/dev/null 2>&1 || true) &
else
  echo "[信息] 请手动在浏览器中打开网页: $WEB_FILE"
fi
ros2 launch "$PKG_NAME" qa_assistant_launch.py
