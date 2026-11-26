#!/usr/bin/env bash
# One-click isolated deployment script for experimental line_follower features.
# Syncs local experimental code, headers, presets, and optional v2 BPU model to
# an isolated car directory (default: /home/sunrise/aic_car_final_exp) and builds
# line_follower there without touching the baseline (/home/sunrise/aic_car_final).
set -Eeuo pipefail

ROOT="${AIC_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
CAR_HOST="${CAR_HOST:-sunrise@172.20.10.2}"
CAR_BASE_DIR="${CAR_BASE_DIR:-/home/sunrise/aic_car_final}"
CAR_EXP_DIR="${CAR_EXP_DIR:-/home/sunrise/aic_car_final_exp}"

DRY_RUN=0
SKIP_MODEL=0
NO_BUILD=0

usage() {
  cat <<EOF
用法: $(basename "$0") [选项]

将实验版代码、4 个头文件、EXP_01~EXP_11 预设、启动脚本、YOLOv8 节点与 v2 BPU 模型
同步到小车隔离实验目录（默认: ${CAR_EXP_DIR}），并在该目录执行一次性 colcon build。
绝不修改现车基线目录（${CAR_BASE_DIR}）。

选项:
  --host <user@ip>       指定小车 SSH 目标（默认: ${CAR_HOST}）
  --exp-dir <remote_dir> 指定小车隔离实验目录（默认: ${CAR_EXP_DIR}）
  --dry-run              仅校验本地文件并打印计划操作，不发起 SSH/SCP 连接
  --skip-model           跳过传输 vision_pipeline/best_modified_yolov8_v2.bin
  --no-build             只同步文件，不在车端执行 colcon build
  -h, --help             显示本帮助信息
EOF
}

normalize_path() {
  local p="$1"
  while [[ "$p" != "/" && "$p" == */ ]]; do
    p="${p%/}"
  done
  printf '%s' "$p"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      [[ $# -ge 2 ]] || { echo "错误: --host 缺少参数" >&2; exit 2; }
      CAR_HOST="$2"
      shift 2
      ;;
    --exp-dir)
      [[ $# -ge 2 ]] || { echo "错误: --exp-dir 缺少参数" >&2; exit 2; }
      CAR_EXP_DIR="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --skip-model)
      SKIP_MODEL=1
      shift
      ;;
    --no-build)
      NO_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误: 未知参数 '$1'" >&2
      usage >&2
      exit 2
      ;;
  esac
done

CAR_BASE_DIR="$(normalize_path "$CAR_BASE_DIR")"
CAR_EXP_DIR="$(normalize_path "$CAR_EXP_DIR")"

# 基线保护：绝不允许覆盖或污染现车基线目录
if [[ -z "$CAR_EXP_DIR" || "$CAR_EXP_DIR" == "/" || \
      "$CAR_EXP_DIR" == "$CAR_BASE_DIR" || \
      "$CAR_EXP_DIR" == "/home/sunrise/aic_car_final" || \
      "$CAR_EXP_DIR" == "~/aic_car_final" || \
      "$CAR_EXP_DIR" == "\$HOME/aic_car_final" ]]; then
  echo "安全拦截: 目标目录 '$CAR_EXP_DIR' 会覆盖或等同于现车基线目录，已中止。" >&2
  exit 1
fi

command -v ssh >/dev/null 2>&1 || { echo "错误: 找不到 ssh" >&2; exit 1; }
command -v scp >/dev/null 2>&1 || { echo "错误: 找不到 scp" >&2; exit 1; }

PKG_DIR="$ROOT/src/line_follow_ws/src/line_follower"
INC_DIR="$PKG_DIR/include/line_follower"
SRC_DIR="$PKG_DIR/src"
LAUNCH_FILE="$PKG_DIR/launch/line_follower_with_detection.launch.py"
BASE_YAML="$PKG_DIR/config/line_follower_params.yaml"
PRESET_DIR="$PKG_DIR/config/parameter_presets"
LAUNCHER_SCRIPT="$ROOT/src/scripts/start_line_following.sh"
YOLO_SCRIPT="$ROOT/src/yolov8/bpu_yolov8_node.py"
MODEL_BIN="$ROOT/vision_pipeline/best_modified_yolov8_v2.bin"

HEADERS=(
  "$INC_DIR/line_follower_node.hpp"
  "$INC_DIR/digit_decision_logic.hpp"
  "$INC_DIR/red_side_selector.hpp"
  "$INC_DIR/steering_pd_controller.hpp"
)
SOURCES=(
  "$SRC_DIR/line_follower_node.cpp"
  "$SRC_DIR/detection_turn_controller.cpp"
)

shopt -s nullglob
EXP_PRESETS=("$PRESET_DIR"/EXP_*.yaml)
shopt -u nullglob

# 本地预检：发起网络传输前确保所有必需文件完备
for f in "${HEADERS[@]}" "${SOURCES[@]}" "$LAUNCH_FILE" "$BASE_YAML" "$LAUNCHER_SCRIPT" "$YOLO_SCRIPT"; do
  [[ -f "$f" ]] || { echo "错误: 缺少必需文件: $f" >&2; exit 1; }
done

if [[ ${#EXP_PRESETS[@]} -eq 0 ]]; then
  echo "错误: 未找到任何实验预设 EXP_*.yaml（检查 $PRESET_DIR）" >&2
  exit 1
fi

if [[ $SKIP_MODEL -eq 0 && ! -f "$MODEL_BIN" ]]; then
  echo "错误: 缺少 v2 BPU 模型文件: $MODEL_BIN（若暂不传模型可加 --skip-model）" >&2
  exit 1
fi

REMOTE_PKG="$CAR_EXP_DIR/src/line_follow_ws/src/line_follower"
REMOTE_INC="$REMOTE_PKG/include/line_follower"
REMOTE_SRC="$REMOTE_PKG/src"
REMOTE_LAUNCH="$REMOTE_PKG/launch"
REMOTE_CFG="$REMOTE_PKG/config"
REMOTE_PRESETS="$REMOTE_CFG/parameter_presets"
REMOTE_SCRIPTS="$CAR_EXP_DIR/src/scripts"
REMOTE_YOLO="$CAR_EXP_DIR/src/yolov8"
REMOTE_MODEL_DIR="$REMOTE_YOLO/mod1"

echo "=== 实验隔离部署配置 ==="
echo "  本地仓库: $ROOT"
echo "  目标主机: $CAR_HOST"
echo "  车端基线: $CAR_BASE_DIR (只读克隆源，绝不修改)"
echo "  隔离目录: $CAR_EXP_DIR"
echo "  实验预设: ${#EXP_PRESETS[@]} 份 (EXP_*.yaml)"
echo "  v2 模型 : $([[ $SKIP_MODEL -eq 0 ]] && echo "同步 ($MODEL_BIN)" || echo "跳过 (--skip-model)")"
echo "  车端编译: $([[ $NO_BUILD -eq 0 ]] && echo "执行 colcon build" || echo "跳过 (--no-build)")"

if [[ $DRY_RUN -eq 1 ]]; then
  echo ""
  echo "[DRY-RUN] 1) 远端克隆与建目录:"
  echo "  若不存在 ${CAR_EXP_DIR} 则 cp -a ${CAR_BASE_DIR} -> ${CAR_EXP_DIR}，并清理复制过来的 line_follow_ws/{build,install,log}"
  echo "  mkdir -p $REMOTE_INC $REMOTE_SRC $REMOTE_LAUNCH $REMOTE_PRESETS $REMOTE_SCRIPTS $REMOTE_MODEL_DIR"
  echo "[DRY-RUN] 2) 传输头文件 (${#HEADERS[@]} 个) -> $REMOTE_INC/"
  for f in "${HEADERS[@]}"; do echo "    - ${f#"$ROOT"/}"; done
  echo "[DRY-RUN] 3) 传输源码 (${#SOURCES[@]} 个) -> $REMOTE_SRC/"
  for f in "${SOURCES[@]}"; do echo "    - ${f#"$ROOT"/}"; done
  echo "[DRY-RUN] 4) 传输 launch / 基础 YAML / 启动脚本 / YOLO 脚本:"
  echo "    - ${LAUNCH_FILE#"$ROOT"/} -> $REMOTE_LAUNCH/"
  echo "    - ${BASE_YAML#"$ROOT"/} -> $REMOTE_CFG/"
  echo "    - ${LAUNCHER_SCRIPT#"$ROOT"/} -> $REMOTE_SCRIPTS/start_line_following.sh"
  echo "    - ${YOLO_SCRIPT#"$ROOT"/} -> $REMOTE_YOLO/bpu_yolov8_node.py"
  echo "[DRY-RUN] 5) 传输实验预设 (${#EXP_PRESETS[@]} 份) -> $REMOTE_PRESETS/"
  for f in "${EXP_PRESETS[@]}"; do echo "    - $(basename "$f")"; done
  if [[ $SKIP_MODEL -eq 0 ]]; then
    echo "[DRY-RUN] 6) 传输 v2 BPU 模型 -> $REMOTE_MODEL_DIR/"
    echo "    - ${MODEL_BIN#"$ROOT"/}"
  fi
  echo "[DRY-RUN] 7) 设置可执行权限: chmod +x $REMOTE_SCRIPTS/start_line_following.sh"
  if [[ $NO_BUILD -eq 0 ]]; then
    echo "[DRY-RUN] 8) 车端一次性编译:"
    echo "    cd $CAR_EXP_DIR/src/line_follow_ws && source /opt/tros/humble/setup.bash && colcon build --packages-select line_follower --symlink-install"
  fi
  echo ""
  echo "[DRY-RUN] 检查通过，未建立任何网络连接。"
  exit 0
fi

SSH_CTL="/tmp/aic_exp_ssh_%r@%h:%p"
SSH_OPTS=(
  -o "ControlMaster=auto"
  -o "ControlPath=$SSH_CTL"
  -o "ControlPersist=60"
  -o "ConnectTimeout=5"
)

cleanup_ssh() {
  ssh -o "ControlPath=$SSH_CTL" -O exit "$CAR_HOST" >/dev/null 2>&1 || true
}
trap cleanup_ssh EXIT

printf -v Q_BASE '%q' "$CAR_BASE_DIR"
printf -v Q_EXP '%q' "$CAR_EXP_DIR"
printf -v Q_INC '%q' "$REMOTE_INC"
printf -v Q_SRC '%q' "$REMOTE_SRC"
printf -v Q_LAUNCH '%q' "$REMOTE_LAUNCH"
printf -v Q_PRESETS '%q' "$REMOTE_PRESETS"
printf -v Q_SCRIPTS '%q' "$REMOTE_SCRIPTS"
printf -v Q_MODEL_DIR '%q' "$REMOTE_MODEL_DIR"

echo ""
echo "[1/5] 准备车端隔离目录并校验基线..."
ssh "${SSH_OPTS[@]}" "$CAR_HOST" "
  set -e
  if [ ! -d $Q_EXP ]; then
    if [ ! -d $Q_BASE ]; then
      echo \"错误: 车端基线目录不存在: $CAR_BASE_DIR\" >&2
      exit 1
    fi
    cp -a $Q_BASE $Q_EXP
    rm -rf $Q_EXP/src/line_follow_ws/build $Q_EXP/src/line_follow_ws/install $Q_EXP/src/line_follow_ws/log
  fi
  mkdir -p $Q_INC $Q_SRC $Q_LAUNCH $Q_PRESETS $Q_SCRIPTS $Q_MODEL_DIR
"

echo "[2/5] 同步头文件、源码、launch 与配置..."
scp "${SSH_OPTS[@]}" "${HEADERS[@]}" "${CAR_HOST}:${REMOTE_INC}/"
scp "${SSH_OPTS[@]}" "${SOURCES[@]}" "${CAR_HOST}:${REMOTE_SRC}/"
scp "${SSH_OPTS[@]}" "$LAUNCH_FILE" "${CAR_HOST}:${REMOTE_LAUNCH}/"
scp "${SSH_OPTS[@]}" "$BASE_YAML" "${CAR_HOST}:${REMOTE_CFG}/"

echo "[3/5] 同步 ${#EXP_PRESETS[@]} 份 EXP_*.yaml 预设、启动脚本与 YOLO 节点..."
scp "${SSH_OPTS[@]}" "${EXP_PRESETS[@]}" "${CAR_HOST}:${REMOTE_PRESETS}/"
scp "${SSH_OPTS[@]}" "$LAUNCHER_SCRIPT" "${CAR_HOST}:${REMOTE_SCRIPTS}/start_line_following.sh"
scp "${SSH_OPTS[@]}" "$YOLO_SCRIPT" "${CAR_HOST}:${REMOTE_YOLO}/bpu_yolov8_node.py"

if [[ $SKIP_MODEL -eq 0 ]]; then
  echo "[4/5] 同步 v2 BPU 模型 ($(basename "$MODEL_BIN"))..."
  scp "${SSH_OPTS[@]}" "$MODEL_BIN" "${CAR_HOST}:${REMOTE_MODEL_DIR}/"
else
  echo "[4/5] 已跳过 v2 模型同步 (--skip-model)"
fi

ssh "${SSH_OPTS[@]}" "$CAR_HOST" "chmod +x $Q_SCRIPTS/start_line_following.sh"

if [[ $NO_BUILD -eq 0 ]]; then
  echo "[5/5] 在车端隔离目录执行一次性 colcon build..."
  printf -v Q_WS '%q' "$CAR_EXP_DIR/src/line_follow_ws"
  printf -v Q_CMAKE_MATCH '%q' "CMAKE_HOME_DIRECTORY:INTERNAL=${CAR_EXP_DIR}/"
  ssh "${SSH_OPTS[@]}" "$CAR_HOST" "
    set -e
    cd $Q_WS
    if [ -f build/line_follower/CMakeCache.txt ] && ! grep -Fq $Q_CMAKE_MATCH build/line_follower/CMakeCache.txt; then
      echo '检测到旧路径 CMakeCache，清理 build/line_follower、install/line_follower 与 log...'
      rm -rf build/line_follower install/line_follower log
    fi
    set +u
    source /opt/tros/humble/setup.bash
    set -u
    colcon build --packages-select line_follower --symlink-install
  "
else
  echo "[5/5] 已跳过车端编译 (--no-build)"
fi

echo ""
echo "部署完成！现车基线 ($CAR_BASE_DIR) 未受任何改动。"
echo "请在车端按下面命令先复测 0.30 m/s 基线（默认全关）："
echo "  ssh $CAR_HOST"
echo "  cd $CAR_EXP_DIR && bash src/scripts/start_line_following.sh --params line_follower_params.yaml"
