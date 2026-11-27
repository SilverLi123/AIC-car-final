#!/usr/bin/env bash
# Deploy the unified dual-map AIC baseline from a teammate's laptop to the RDK X5
# and compile `line_follower` in Release mode in a single command.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="sunrise@172.20.10.3"
REMOTE_DIR="/home/sunrise/aic_car_v2_20261126"
DRY_RUN=false

usage() {
  cat <<'EOF'
用法：
  bash src/scripts/deploy_to_car.sh [sunrise@<CAR_IP>] [远程目录] [--dry-run]

示例：
  bash src/scripts/deploy_to_car.sh
  bash src/scripts/deploy_to_car.sh sunrise@172.20.10.3 /home/sunrise/aic_car_v2_20261126
  bash src/scripts/deploy_to_car.sh sunrise@192.168.137.10 --dry-run

说明：
  1) 兼容 macOS / Linux / Windows Git Bash 或 WSL（优先 rsync，自动降级为 tar|ssh）
  2) 若车端已有工作目录，会自动在 $REMOTE_DIR/backups/ 下生成带时间戳的备份包
  3) 若车端尚无工作目录，会自动初始化完整结构并同步底盘 car_ws 与相机脚本
  4) 自动清理旧 CMakeCache 冲突并以 Release 模式编译 line_follower
EOF
}

POS_ARGS=()
while (($#)); do
  case "$1" in
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    --*) echo "未知选项：$1" >&2; usage >&2; exit 2 ;;
    *) POS_ARGS+=("$1"); shift ;;
  esac
done

if ((${#POS_ARGS[@]} >= 1)); then HOST="${POS_ARGS[0]}"; fi
if ((${#POS_ARGS[@]} >= 2)); then REMOTE_DIR="${POS_ARGS[1]}"; fi
if ((${#POS_ARGS[@]} > 2)); then echo "参数过多：${POS_ARGS[*]}" >&2; usage >&2; exit 2; fi

LOCAL_LF="$ROOT/src/line_follow_ws/src/line_follower"
LOCAL_SCRIPTS="$ROOT/src/scripts"
LOCAL_YOLO="$ROOT/src/yolov8/bpu_yolov8_node.py"
LOCAL_YOLO_BIN="$ROOT/src/yolov8/mod1/best_modified_yolov8.bin"

[[ -d "$LOCAL_LF" ]] || { echo "缺少本地 line_follower：$LOCAL_LF" >&2; exit 2; }
[[ -f "$LOCAL_YOLO" ]] || { echo "缺少本地 YOLO 节点：$LOCAL_YOLO" >&2; exit 2; }

echo "========================================"
echo "AIC 统一双图基线一键部署"
echo "本地仓库：$ROOT"
echo "目标车端：$HOST:$REMOTE_DIR"
echo "演练模式：$DRY_RUN"
echo "========================================"

if [[ "$DRY_RUN" == true ]]; then
  echo "[DRY-RUN] 跳过 SSH 连接、同步与车端编译。将同步以下内容："
  echo "  - $LOCAL_LF -> $REMOTE_DIR/src/line_follow_ws/src/line_follower"
  echo "  - $LOCAL_SCRIPTS -> $REMOTE_DIR/src/scripts"
  echo "  - $LOCAL_YOLO & mod1 .bin -> $REMOTE_DIR/src/yolov8/"
  echo "  - yolo/ (v3/v5 .bin + README.md) -> $REMOTE_DIR/yolo/"
  echo "  - README.md & handoff.md -> $REMOTE_DIR/"
  exit 0
fi

echo "=== [1/5] 检查车端 SSH 连通性 ==="
ssh -o ConnectTimeout=5 "$HOST" "true"

echo "=== [2/5] 备份已有代码并初始化远程目录 ==="
FRESH_INIT="$(ssh -o ConnectTimeout=5 "$HOST" "REMOTE_DIR='$REMOTE_DIR' bash -s" << 'EOF'
set -e
if [[ -d "$REMOTE_DIR/src/line_follow_ws/src/line_follower" ]]; then
  mkdir -p "$REMOTE_DIR/backups" "$REMOTE_DIR/src/yolov8/mod1" "$REMOTE_DIR/yolo"
  STAMP="$(date +%Y%m%d_%H%M%S)"
  BAK="$REMOTE_DIR/backups/pre_deploy_${STAMP}.tar.gz"
  tar -C "$REMOTE_DIR" -czf "$BAK" \
    --exclude='*.bak*' \
    --exclude='__pycache__' \
    src/line_follow_ws/src/line_follower \
    src/scripts \
    $( [[ -d "$REMOTE_DIR/src/yolov8" ]] && echo src/yolov8 )
  echo "EXISTING"
else
  mkdir -p "$REMOTE_DIR/backups" "$REMOTE_DIR/src/line_follow_ws/src" "$REMOTE_DIR/src/scripts" \
           "$REMOTE_DIR/src/yolov8/mod1" "$REMOTE_DIR/yolo"
  echo "FRESH"
fi
EOF
)"
echo "远程工作区状态：$FRESH_INIT"

sync_dir_tar() {
  local src_dir="$1"
  local dst_dir="$2"
  tar -C "$src_dir" \
    --exclude='*.bak*' \
    --exclude='__pycache__' \
    --exclude='.DS_Store' \
    --exclude='*.pt' \
    --exclude='*.log' \
    -cf - . | ssh -o ConnectTimeout=5 "$HOST" "mkdir -p '$dst_dir' && tar -C '$dst_dir' -xf -"
}

USE_RSYNC=false
if command -v rsync >/dev/null 2>&1 && ssh -o ConnectTimeout=5 "$HOST" "command -v rsync >/dev/null 2>&1"; then
  USE_RSYNC=true
fi

echo "=== [3/5] 同步统一基线源码、预设、脚本与 YOLO 模型（rsync=$USE_RSYNC） ==="
if [[ "$USE_RSYNC" == true ]]; then
  rsync -az --delete \
    --exclude '*.bak*' \
    --exclude '__pycache__/' \
    --exclude '.DS_Store' \
    "$LOCAL_LF/" \
    "$HOST:$REMOTE_DIR/src/line_follow_ws/src/line_follower/"
  rsync -az \
    --exclude '__pycache__/' \
    --exclude '.DS_Store' \
    "$LOCAL_SCRIPTS/" \
    "$HOST:$REMOTE_DIR/src/scripts/"
  rsync -az "$LOCAL_YOLO" "$HOST:$REMOTE_DIR/src/yolov8/bpu_yolov8_node.py"
  [[ -f "$LOCAL_YOLO_BIN" ]] && rsync -az "$LOCAL_YOLO_BIN" "$HOST:$REMOTE_DIR/src/yolov8/mod1/best_modified_yolov8.bin"
  rsync -az "$ROOT/README.md" "$HOST:$REMOTE_DIR/README.md"
  [[ -f "$ROOT/handoff.md" ]] && rsync -az "$ROOT/handoff.md" "$HOST:$REMOTE_DIR/handoff.md"
  if [[ -d "$ROOT/yolo" ]]; then
    rsync -az \
      --include 'README.md' \
      --include 'v3_yolov8n/' --include 'v3_yolov8n/*.bin' \
      --include 'v5_yolov8s/' --include 'v5_yolov8s/*.bin' --include 'v5_yolov8s/*.json' \
      --exclude '*' \
      "$ROOT/yolo/" "$HOST:$REMOTE_DIR/yolo/"
  fi
  if [[ "$FRESH_INIT" == "FRESH" ]]; then
    [[ -d "$ROOT/src/camera" ]] && rsync -az --exclude '__pycache__/' "$ROOT/src/camera/" "$HOST:$REMOTE_DIR/src/camera/"
    [[ -d "$ROOT/src/car_ws" ]] && rsync -az --exclude 'build/' --exclude 'install/' --exclude 'log/' "$ROOT/src/car_ws/" "$HOST:$REMOTE_DIR/src/car_ws/"
  fi
else
  ssh -o ConnectTimeout=5 "$HOST" "rm -rf '$REMOTE_DIR/src/line_follow_ws/src/line_follower/config/parameter_presets'"
  sync_dir_tar "$LOCAL_LF" "$REMOTE_DIR/src/line_follow_ws/src/line_follower"
  sync_dir_tar "$LOCAL_SCRIPTS" "$REMOTE_DIR/src/scripts"
  scp -o ConnectTimeout=5 "$LOCAL_YOLO" "$HOST:$REMOTE_DIR/src/yolov8/bpu_yolov8_node.py"
  [[ -f "$LOCAL_YOLO_BIN" ]] && scp -o ConnectTimeout=5 "$LOCAL_YOLO_BIN" "$HOST:$REMOTE_DIR/src/yolov8/mod1/best_modified_yolov8.bin"
  scp -o ConnectTimeout=5 "$ROOT/README.md" "$HOST:$REMOTE_DIR/README.md"
  [[ -f "$ROOT/handoff.md" ]] && scp -o ConnectTimeout=5 "$ROOT/handoff.md" "$HOST:$REMOTE_DIR/handoff.md"
  if [[ -d "$ROOT/yolo" ]]; then
    tar -C "$ROOT/yolo" \
      --exclude='*.pt' --exclude='*.log' \
      --exclude='v0_orig_11m' --exclude='v1_yolov8n' --exclude='v2_yolov8n' \
      -cf - . | ssh -o ConnectTimeout=5 "$HOST" "mkdir -p '$REMOTE_DIR/yolo' && tar -C '$REMOTE_DIR/yolo' -xf -"
  fi
  if [[ "$FRESH_INIT" == "FRESH" ]]; then
    [[ -d "$ROOT/src/camera" ]] && sync_dir_tar "$ROOT/src/camera" "$REMOTE_DIR/src/camera"
    [[ -d "$ROOT/src/car_ws" ]] && sync_dir_tar "$ROOT/src/car_ws" "$REMOTE_DIR/src/car_ws"
  fi
fi

echo "=== [4/5] 车端清理冲突缓存并编译 Release ==="
ssh -o ConnectTimeout=5 "$HOST" "REMOTE_DIR='$REMOTE_DIR' bash -s" << 'EOF'
set -Eeuo pipefail
TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"
chmod +x "$REMOTE_DIR/src/scripts/"*.sh "$REMOTE_DIR/src/scripts/"*.py
set +u
source "$TROS_SETUP"
set -u

if [[ ! -f "$REMOTE_DIR/src/car_ws/install/setup.bash" && -d "$REMOTE_DIR/src/car_ws/src" ]]; then
  echo "Building car_ws (first-time init)..."
  cd "$REMOTE_DIR/src/car_ws"
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

cd "$REMOTE_DIR/src/line_follow_ws"
CACHE_FILE="build/line_follower/CMakeCache.txt"
if [[ -f "$CACHE_FILE" ]]; then
  CACHED_SRC="$(grep -m1 '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$CACHE_FILE" | cut -d= -f2 || true)"
  EXPECTED_SRC="$REMOTE_DIR/src/line_follow_ws/src/line_follower"
  if [[ -n "$CACHED_SRC" && "$CACHED_SRC" != "$EXPECTED_SRC" ]]; then
    echo "Stale CMakeCache detected ($CACHED_SRC != $EXPECTED_SRC), cleaning build/install..."
    rm -rf build/line_follower install/line_follower
  fi
fi

colcon build --packages-select line_follower --cmake-args -DCMAKE_BUILD_TYPE=Release
EOF

echo "=== [5/5] 校验车端产物与预设 ==="
ssh -o ConnectTimeout=5 "$HOST" "REMOTE_DIR='$REMOTE_DIR' bash -s" << 'EOF'
set -Eeuo pipefail
LIB_DIR="$REMOTE_DIR/src/line_follow_ws/install/line_follower/lib/line_follower"
CFG_DIR="$REMOTE_DIR/src/line_follow_ws/install/line_follower/share/line_follower/config/parameter_presets"
for bin in line_follower_node detection_turn_controller cmd_mux_node start_toggle_node; do
  [[ -x "$LIB_DIR/$bin" ]] || { echo "Missing installed binary: $LIB_DIR/$bin" >&2; exit 1; }
done
for yaml_file in "$CFG_DIR/YOLO_RING_BASE.yaml" "$CFG_DIR/YOLO_FORK_BASE.yaml"; do
  [[ -f "$yaml_file" ]] || { echo "Missing installed preset: $yaml_file" >&2; exit 1; }
done
cd "$REMOTE_DIR"
bash src/scripts/start_line_following.sh --list-models
EOF

echo ""
echo "部署与编译全部成功！明日上车可直接运行："
echo "  # 1) 环岛地图（Ring Verified 0.52 m/s, 30.43-31.96s）："
echo "  cd $REMOTE_DIR && bash src/scripts/start_line_following.sh --map ring --params YOLO_RING_BASE.yaml --yolo --yolo-model best_modified_yolov8_v3.bin --record-tag RING_BASE"
echo ""
echo "  # 2) 岔路地图（Fork Verified FORK_GAP_FIX1 基线）："
echo "  cd $REMOTE_DIR && bash src/scripts/start_line_following.sh --map fork --params YOLO_FORK_BASE.yaml --yolo --yolo-model best_modified_yolov8_v3.bin --record-tag FORK_BASE"
