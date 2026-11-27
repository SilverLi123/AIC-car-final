#!/usr/bin/env bash
# Record a compact zstd-compressed ROS 2 bag with experiment metadata (run_meta.json).
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"
if [[ -z "${BAG_DIR:-}" ]]; then
  if command -v mountpoint >/dev/null 2>&1 && mountpoint -q /mnt/aic_usb && [[ -w /mnt/aic_usb ]]; then
    BAG_DIR="/mnt/aic_usb/aic_bags_$(date +%Y%m%d)"
  else
    BAG_DIR="$ROOT/recorded_bags"
  fi
fi

TAG="run"
MAP=""
PARAMS=""
MODEL=""
NOTE=""
OUT_NAME=""
INCLUDE_DEBUG_IMAGE=false
WAIT_ENTER=false
MIN_FREE_MB=1500
AUTO_ANALYZE=false

usage() {
  cat <<'EOF'
用法：
  bash src/scripts/record_bag.sh [选项]

选项：
  --tag <名称>           实验标签，用于自动命名录包目录（默认 run）
  --map ring|fork        赛道地图（可选：ring / fork）
  --params <YAML路径>    本次使用的参数 YAML（记录路径与 SHA-256）
  --model <bin路径>      本次使用的 YOLO 模型（记录路径、大小与 MD5）
  --note <文本>          补充实验备注（例如 "S060_LATCAP 第2圈"）
  --out-name <目录名>    自定义输出目录名（默认 <map>_<tag>_YYYYMMDD_HHMMSS）
  --bag-dir <目录>       录包保存根目录（优先可写 U 盘 /mnt/aic_usb/aic_bags_YYYYMMDD，否则 <repo>/recorded_bags）
  --debug-image          同时录制 /line_follower/debug_image（体积显著增大，默认关闭）
  --wait-enter           启动后按回车再开始录包
  --min-free-mb <MB>     磁盘最低可用空间检查阈值（默认 1500 MB）
  --analyze              录包退出后自动调用 analyze_bag.py 输出分析摘要
  -h, --help             显示帮助

按 Ctrl+C 停止录包，脚本会自动刷新 metadata.yaml 并写入 run_meta.json。
EOF
}

sanitize_name() {
  local raw="$1" cleaned
  cleaned="$(printf '%s' "$raw" | sed 's/[^A-Za-z0-9._-]/_/g; s/__*/_/g; s/^_*//; s/_*$//')"
  printf '%s\n' "${cleaned:-run}"
}

abs_path() {
  local p="$1"
  if [[ -e "$p" ]]; then
    if [[ -d "$p" ]]; then
      (cd "$p" && pwd -P)
    else
      printf '%s/%s\n' "$(cd "$(dirname "$p")" && pwd -P)" "$(basename "$p")"
    fi
  else
    printf '%s\n' "$p"
  fi
}

while (($#)); do
  case "$1" in
    --tag)
      (($# >= 2)) || { echo "--tag 需要名称" >&2; exit 2; }
      TAG="$2"; shift 2 ;;
    --map)
      (($# >= 2)) || { echo "--map 需要 ring 或 fork" >&2; exit 2; }
      case "$2" in
        ring|roundabout|环岛) MAP="ring" ;;
        fork|crossroad|岔路) MAP="fork" ;;
        *) echo "不支持的地图：$2（可选 ring 或 fork）" >&2; exit 2 ;;
      esac
      shift 2 ;;
    --params)
      (($# >= 2)) || { echo "--params 需要 YAML 路径" >&2; exit 2; }
      PARAMS="$2"; shift 2 ;;
    --model)
      (($# >= 2)) || { echo "--model 需要 .bin 模型路径" >&2; exit 2; }
      MODEL="$2"; shift 2 ;;
    --note)
      (($# >= 2)) || { echo "--note 需要文本" >&2; exit 2; }
      NOTE="$2"; shift 2 ;;
    --out-name)
      (($# >= 2)) || { echo "--out-name 需要目录名" >&2; exit 2; }
      OUT_NAME="$2"; shift 2 ;;
    --bag-dir)
      (($# >= 2)) || { echo "--bag-dir 需要目录" >&2; exit 2; }
      BAG_DIR="$2"; shift 2 ;;
    --debug-image)
      INCLUDE_DEBUG_IMAGE=true; shift ;;
    --wait-enter)
      WAIT_ENTER=true; shift ;;
    --min-free-mb)
      (($# >= 2)) || { echo "--min-free-mb 需要整数" >&2; exit 2; }
      MIN_FREE_MB="$2"; shift 2 ;;
    --analyze)
      AUTO_ANALYZE=true; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$MIN_FREE_MB" =~ ^[0-9]+$ ]] || { echo "--min-free-mb 必须为非负整数：$MIN_FREE_MB" >&2; exit 2; }

if [[ -n "$PARAMS" && -e "$PARAMS" ]]; then
  PARAMS="$(abs_path "$PARAMS")"
fi
if [[ -n "$MODEL" && -e "$MODEL" ]]; then
  MODEL="$(abs_path "$MODEL")"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  if [[ -f "$TROS_SETUP" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$TROS_SETUP" >/dev/null 2>&1 || true
    set -u
  fi
fi
command -v ros2 >/dev/null 2>&1 || {
  echo "未找到 ros2 命令，请确认已安装 ROS 2 或设置 TROS_SETUP=$TROS_SETUP" >&2
  exit 1
}

mkdir -p "$BAG_DIR"
BAG_DIR="$(abs_path "$BAG_DIR")"

if ((MIN_FREE_MB > 0)); then
  avail_mb="$(df -Pm "$BAG_DIR" | awk 'NR==2 { print $4 }')"
  if [[ "$avail_mb" =~ ^[0-9]+$ ]] && ((avail_mb < MIN_FREE_MB)); then
    echo "磁盘可用空间不足：${avail_mb} MB < ${MIN_FREE_MB} MB（目录：$BAG_DIR）" >&2
    exit 1
  fi
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
SAFE_TAG="$(sanitize_name "$TAG")"
if [[ -n "$OUT_NAME" ]]; then
  BAG_NAME="$(sanitize_name "$OUT_NAME")"
elif [[ -n "$MAP" ]]; then
  BAG_NAME="${MAP}_${SAFE_TAG}_${STAMP}"
else
  BAG_NAME="${SAFE_TAG}_${STAMP}"
fi
BAG_PATH="$BAG_DIR/$BAG_NAME"
if [[ -e "$BAG_PATH" ]]; then
  BAG_NAME="${BAG_NAME}_$$"
  BAG_PATH="$BAG_DIR/$BAG_NAME"
fi

TOPICS=(
  /image_raw
  /odom
  /cmd_vel
  /cmd_vel_follower
  /cmd_vel_avoid
  /cmd_vel_detection
  /cmd_vel_stop
  /hobot_dnn_detection
  /line_follower/track_state
  /line_follower/centroid
  /line_follower/start_cmd
  /imu
  /imu_data
  /tf
  /tf_static
  /rosout
)
if [[ "$INCLUDE_DEBUG_IMAGE" == true ]]; then
  TOPICS+=(/line_follower/debug_image)
fi

if [[ "$WAIT_ENTER" == true ]]; then
  echo "录包已就绪：$BAG_PATH"
  read -r -p "按回车开始录包..." _
fi

STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
START_EPOCH="$(date +%s)"
REC_PID=""
FINALIZED=false

write_run_meta() {
  local ended_at end_epoch duration_sec git_commit
  ended_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  end_epoch="$(date +%s)"
  duration_sec=$((end_epoch - START_EPOCH))
  git_commit="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || true)"
  [[ -d "$BAG_PATH" ]] || return 0
  command -v python3 >/dev/null 2>&1 || return 0

  META_BAG_PATH="$BAG_PATH" \
  META_BAG_NAME="$BAG_NAME" \
  META_MAP="$MAP" \
  META_TAG="$TAG" \
  META_NOTE="$NOTE" \
  META_PARAMS="$PARAMS" \
  META_MODEL="$MODEL" \
  META_DEBUG_IMAGE="$INCLUDE_DEBUG_IMAGE" \
  META_GIT_COMMIT="$git_commit" \
  META_STARTED_AT="$STARTED_AT" \
  META_ENDED_AT="$ended_at" \
  META_DURATION_SEC="$duration_sec" \
  python3 - << 'PYEOF'
import hashlib
import json
import os
from pathlib import Path
import socket

bag_path = Path(os.environ["META_BAG_PATH"])

def file_hashes(path_str: str):
    if not path_str:
        return "", "", "", "", 0
    p = Path(path_str)
    if not p.is_file():
        return path_str, p.name, "", "", 0
    md5 = hashlib.md5()
    sha256 = hashlib.sha256()
    size = 0
    with p.open("rb") as f:
        while True:
            chunk = f.read(1024 * 1024)
            if not chunk:
                break
            md5.update(chunk)
            sha256.update(chunk)
            size += len(chunk)
    return str(p), p.name, md5.hexdigest(), sha256.hexdigest(), size

params_raw = os.environ.get("META_PARAMS", "")
params_path, params_base, _, params_sha, _ = (
    file_hashes(params_raw) if params_raw else ("", "", "", "", 0)
)
if params_raw and not params_path:
    params_path = params_raw
    params_base = Path(params_raw).name

model_raw = os.environ.get("META_MODEL", "")
model_path, model_base, model_md5, _, model_size = (
    file_hashes(model_raw) if model_raw else ("", "", "", "", 0)
)
if model_raw and not model_path:
    model_path = model_raw
    model_base = Path(model_raw).name

db3_files = sorted(p.name for p in bag_path.glob("*.db3"))
db3_bytes = sum(p.stat().st_size for p in bag_path.glob("*.db3") if p.is_file())

meta = {
    "bag_name": os.environ["META_BAG_NAME"],
    "map": os.environ.get("META_MAP", ""),
    "tag": os.environ.get("META_TAG", "run"),
    "note": os.environ.get("META_NOTE", ""),
    "params_file": params_path,
    "params_basename": params_base,
    "params_sha256": params_sha,
    "yolo_model": model_path,
    "yolo_model_basename": model_base,
    "yolo_model_md5": model_md5,
    "yolo_model_size_bytes": model_size,
    "include_debug_image": os.environ.get("META_DEBUG_IMAGE") == "true",
    "git_commit": os.environ.get("META_GIT_COMMIT", ""),
    "started_at": os.environ.get("META_STARTED_AT", ""),
    "ended_at": os.environ.get("META_ENDED_AT", ""),
    "duration_sec": int(os.environ.get("META_DURATION_SEC", "0")),
    "hostname": socket.gethostname(),
    "db3_files": db3_files,
    "db3_total_bytes": db3_bytes,
}

out_file = bag_path / "run_meta.json"
out_file.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PYEOF
}

finalize_bag() {
  if [[ "$FINALIZED" == true ]]; then
    return 0
  fi
  FINALIZED=true

  if [[ -n "$REC_PID" ]] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    local waited=0
    while kill -0 "$REC_PID" 2>/dev/null && ((waited < 50)); do
      sleep 0.1
      ((waited += 1))
    done
    if kill -0 "$REC_PID" 2>/dev/null; then
      kill -TERM "$REC_PID" 2>/dev/null || true
      wait "$REC_PID" 2>/dev/null || true
    else
      wait "$REC_PID" 2>/dev/null || true
    fi
  fi

  write_run_meta

  if [[ -d "$BAG_PATH" ]]; then
    local total_kb total_mb has_db3=false has_meta=false
    if ls "$BAG_PATH"/*.db3 >/dev/null 2>&1; then
      has_db3=true
    fi
    [[ -f "$BAG_PATH/metadata.yaml" ]] && has_meta=true
    total_kb="$(du -sk "$BAG_PATH" | awk '{print $1}')"
    total_mb="$(awk -v kb="$total_kb" 'BEGIN { printf "%.1f", kb / 1024.0 }')"
    echo ""
    echo "录包完成：$BAG_PATH（${total_mb} MB，db3=${has_db3}，metadata.yaml=${has_meta}）"
    echo "离线分析命令："
    echo "  python3 src/scripts/analyze_bag.py \"$BAG_PATH\""
    if [[ "$AUTO_ANALYZE" == true && "$has_db3" == true && -f "$ROOT/src/scripts/analyze_bag.py" ]]; then
      python3 "$ROOT/src/scripts/analyze_bag.py" "$BAG_PATH" || true
    fi
  else
    echo "警告：未生成录包目录 $BAG_PATH" >&2
  fi
}

on_signal() {
  finalize_bag
  exit 0
}

trap finalize_bag EXIT
trap on_signal INT TERM HUP

echo "开始录包："
echo "  目录：$BAG_PATH"
[[ -z "$MAP" ]] || echo "  地图：$MAP"
[[ -z "$TAG" ]] || echo "  标签：$TAG"
[[ -z "$PARAMS" ]] || echo "  参数：$(basename "$PARAMS")"
[[ -z "$MODEL" ]] || echo "  模型：$(basename "$MODEL")"
[[ -z "$NOTE" ]] || echo "  备注：$NOTE"
echo "  压缩：sqlite3 + zstd（${#TOPICS[@]} 个话题，按 Ctrl+C 停止并刷新索引）"

ros2 bag record -s sqlite3 \
  --compression-mode message \
  --compression-format zstd \
  --compression-threads 1 \
  --max-cache-size 536870912 \
  -o "$BAG_PATH" \
  "${TOPICS[@]}" &
REC_PID=$!

set +e
wait "$REC_PID"
rec_rc=$?
set -e
REC_PID=""
finalize_bag
exit "$rec_rc"
