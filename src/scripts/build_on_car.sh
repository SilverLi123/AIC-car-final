#!/usr/bin/env bash
# Build the unified dual-map ROS 2 workspace directly on RDK X5 after `git pull`.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"

if [[ ! -f "$TROS_SETUP" ]]; then
  echo "找不到 TROS/ROS2 环境文件：$TROS_SETUP" >&2
  exit 1
fi

chmod +x "$ROOT/src/scripts/"*.sh "$ROOT/src/scripts/"*.py

set +u
# shellcheck disable=SC1090
source "$TROS_SETUP"
set -u

if [[ ! -f "$ROOT/src/car_ws/install/setup.bash" && -d "$ROOT/src/car_ws/src" ]]; then
  echo "=== [1/3] 首次构建底盘工作区 car_ws ==="
  cd "$ROOT/src/car_ws"
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
else
  echo "=== [1/3] car_ws 已就绪，跳过 ==="
fi

echo "=== [2/3] 构建统一双图 line_follower（Release） ==="
cd "$ROOT/src/line_follow_ws"
CACHE_FILE="build/line_follower/CMakeCache.txt"
if [[ -f "$CACHE_FILE" ]]; then
  CACHED_SRC="$(grep -m1 '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$CACHE_FILE" | cut -d= -f2 || true)"
  EXPECTED_SRC="$ROOT/src/line_follow_ws/src/line_follower"
  if [[ -n "$CACHED_SRC" && "$CACHED_SRC" != "$EXPECTED_SRC" ]]; then
    echo "检测到旧构建路径 ($CACHED_SRC != $EXPECTED_SRC)，清理后重编..."
    rm -rf build/line_follower install/line_follower
  fi
fi
colcon build --packages-select line_follower --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "=== [3/3] 校验可执行文件与统一预设 ==="
LIB_DIR="$ROOT/src/line_follow_ws/install/line_follower/lib/line_follower"
CFG_DIR="$ROOT/src/line_follow_ws/install/line_follower/share/line_follower/config/parameter_presets"
for bin in line_follower_node detection_turn_controller cmd_mux_node start_toggle_node; do
  [[ -x "$LIB_DIR/$bin" ]] || { echo "缺少编译产物：$LIB_DIR/$bin" >&2; exit 1; }
done
for yaml_file in "$CFG_DIR/YOLO_RING_BASE.yaml" "$CFG_DIR/YOLO_FORK_BASE.yaml"; do
  [[ -f "$yaml_file" ]] || { echo "缺少安装预设：$yaml_file" >&2; exit 1; }
done

cd "$ROOT"
bash src/scripts/start_line_following.sh --list-models
bash src/scripts/start_line_following.sh --list-presets

echo ""
echo "机载编译完成！可直接运行："
echo "  # 环岛地图："
echo "  bash src/scripts/start_line_following.sh --map ring --params YOLO_RING_BASE.yaml --yolo --yolo-model best_modified_yolov8_v3.bin --record-tag RING_BASE"
echo "  # 岔路地图："
echo "  bash src/scripts/start_line_following.sh --map fork --params YOLO_FORK_BASE.yaml --yolo --yolo-model best_modified_yolov8_v3.bin --record-tag FORK_BASE"
