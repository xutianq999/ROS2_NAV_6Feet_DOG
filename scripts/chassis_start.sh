#!/usr/bin/env bash
#
# 底盘启动脚本：根据底盘类型拉起对应驱动。
#
# 自动检测：
#   - 检测到 CAN 接口(can0)          -> 原方案：BUNKER MINI 2.0 底盘驱动
#   - 没有 CAN 接口                  -> Dobot Hexplorer (miniHex_v2) 六足适配层
#     适配层订阅导航安全链的 /cmd_vel，维护六足状态机到 WALK，把速度转发给
#     vendor 运控的 /vel_cmd。vendor 运控（robot_controller_release 的 main）
#     由整机上电自启，本脚本不启动也不停止它。
#
# 也可以用环境变量强制指定：
#   CHASSIS_TYPE=hexplorer ./chassis_start.sh
#   CHASSIS_TYPE=bunker    ./chassis_start.sh
#
# 用法：
#   ./chassis_start.sh              # 自动检测并启动
#   ./chassis_start.sh --can-only   # 只配置 can0，不启动驱动（仅 bunker 有效）
#
# 环境变量：
#   CAN_IFACE    默认 can0
#   CAN_BITRATE  默认 500000（Bunker Mini 2.0 用 500k）
#   HEXPLORER_WS vendor ROS2 包路径，默认 ~/robot_controller_release/ros2_packages
#               （提供 custom_msg 消息类型；本脚本只 source，不修改 vendor 内容）

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
HEXPLORER_WS="${HEXPLORER_WS:-$HOME/robot_controller_release/ros2_packages}"
CAN_ONLY=0
[ "${1:-}" = "--can-only" ] && CAN_ONLY=1

log() { printf '[chassis] %s\n' "$*"; }
die() { printf '[chassis] 错误: %s\n' "$*" >&2; exit 1; }

# --- 1. 底盘类型 ------------------------------------------------------------
CHASSIS_TYPE="${CHASSIS_TYPE:-auto}"
if [ "$CHASSIS_TYPE" = "auto" ]; then
  if [ -e "/sys/class/net/${CAN_IFACE}" ]; then
    CHASSIS_TYPE=bunker
  else
    CHASSIS_TYPE=hexplorer
  fi
fi
case "$CHASSIS_TYPE" in
  bunker|hexplorer) ;;
  *) die "CHASSIS_TYPE 只能是 auto / bunker / hexplorer，收到 '$CHASSIS_TYPE'" ;;
esac
log "底盘类型: $CHASSIS_TYPE"

# --- 2. 环境 ----------------------------------------------------------------
set +u
source /opt/ros/humble/setup.bash
set -u

if [ "$CHASSIS_TYPE" = "hexplorer" ]; then
  # --- Hexplorer 六足 -------------------------------------------------------
  [ -d "$HEXPLORER_WS" ] || die "未找到 vendor ROS2 包: $HEXPLORER_WS
（custom_msg 消息定义在里面；机器人断电时该目录也应存在，路径可用 HEXPLORER_WS=... 覆盖）"
  [ "$CAN_ONLY" = 1 ] && { log "--can-only 对六足无意义，忽略。"; exit 0; }

  set +u
  # vendor 的 custom_msg（RobotCommand/RobotState）。只 source，不改任何 vendor 文件。
  # shellcheck disable=SC1091
  source "$HEXPLORER_WS/setup.bash"
  source install/setup.bash
  set -u

  if ! ros2 pkg prefix custom_msg >/dev/null 2>&1; then
    die "source 之后仍找不到 custom_msg，请检查 $HEXPLORER_WS"
  fi

  log "等待 vendor 运控上线（/robot_state 需要整机上电后才会出现）..."
  log "启动 Hexplorer 适配层: /cmd_vel -> /vel_cmd（限幅后转发，状态机自动走到 WALK）"
  exec ros2 run hexplorer_adapter hexplorer_adapter_node \
    --ros-args -r __node:=hexplorer_adapter
fi

# --- 3. Bunker：确保 CAN 接口存在（USB 适配器需要 gs_usb 内核模块）-----------
if [ ! -e "/sys/class/net/${CAN_IFACE}" ]; then
  log "未发现 ${CAN_IFACE}，尝试加载 gs_usb 内核模块..."
  sudo modprobe gs_usb || true
  sleep 1
fi
[ -e "/sys/class/net/${CAN_IFACE}" ] \
  || die "仍未发现 ${CAN_IFACE}，请检查 CAN-USB 适配器是否插好。"

# --- 4. 拉起接口并设置波特率 ----------------------------------------------
# 接口若已 UP 则不重复配置（重复 set 在波特率不一致时会报 busy）。
if ip link show "$CAN_IFACE" | grep -qE '<[^>]*\bUP\b'; then
  log "${CAN_IFACE} 已处于 UP 状态，跳过配置。"
else
  log "拉起 ${CAN_IFACE}（bitrate=${CAN_BITRATE}）..."
  sudo ip link set "$CAN_IFACE" up type can bitrate "$CAN_BITRATE"
fi

# --- 5. 采样确认总线确实有报文 --------------------------------------------
if command -v candump >/dev/null 2>&1; then
  log "采样 ${CAN_IFACE} 2 秒，确认总线上有报文..."
  if timeout 2 candump -n 1 "$CAN_IFACE" >/dev/null 2>&1; then
    log "已收到 CAN 报文，总线正常。"
  else
    log "警告: 2 秒内未收到 CAN 报文，请确认底盘已上电、线缆连接正常。"
  fi
else
  log "未安装 can-utils（缺少 candump），跳过总线自检。"
fi

[ "$CAN_ONLY" = 1 ] && { log "仅配置模式（--can-only），不启动驱动。"; exit 0; }

# --- 6. 启动底盘驱动 ------------------------------------------------------
log "启动 BUNKER MINI 2.0 底盘驱动（bunker_mini2_base.launch.py）..."
# ROS 的 setup.bash 会引用未定义的 COLCON_TRACE，与 set -u 冲突，
# 故 source 期间临时关闭 nounset。
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
exec ros2 launch me_nav2_bringup bunker_mini2_base.launch.py
