#!/usr/bin/env bash
# Select a parameter YAML and start the original line-following chain.
# YOLO is opt-in; without --yolo only the C++ line follower is launched.
set -Eeuo pipefail
set +m

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CAR_WS="$ROOT/src/car_ws"
LINE_WS="${LINE_WS:-$ROOT/src/line_follow_ws}"
CAMERA_DIR="$ROOT/src/camera"
YOLO_DIR="$ROOT/src/yolov8"
CONFIG_DIR="$LINE_WS/src/line_follower/config"
TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"
LOG_ROOT="${LOG_ROOT:-$HOME/rdkx5_logs}"
PARAM_USAGE_FILE="${PARAM_USAGE_FILE:-$LOG_ROOT/parameter_usage.tsv}"
START_CMD_TOPIC="/line_follower/start_cmd"

PARAMS=""
MAP=""
START_YOLO=false
YOLO_MODEL=""
YOLO_ROI_FILTER=false
FINISH_HEADING_CALIBRATION=""
SKIP_CHASSIS=false
PIDS=()
CHASSIS_PID=""
TOGGLE_PID=""
TAIL_PID=""
LOG_DIR=""

usage() {
  cat <<'EOF'
用法：
  bash src/scripts/start_line_following.sh                 菜单选择 YAML，不启动 YOLO
  bash src/scripts/start_line_following.sh --params <YAML> 直接指定参数文件
  bash src/scripts/start_line_following.sh --map ring|fork 选择地图后进入对应参数菜单
  bash src/scripts/start_line_following.sh --yolo           启动 YOLO 数字转向
  bash src/scripts/start_line_following.sh --finish-heading <JSON>  启用终点对正实验

选项：
  --params <路径或文件名>  巡线参数 YAML；直接指定时不弹交互菜单
  --map ring|fork          选择对应地图菜单；与 --params 同用时校验配置归属
  --yolo                   同时启动 YOLO，并使用原版三节点巡线入口
  --yolo-model <路径>      指定 YOLO .bin 模型路径（未指定且存在 mod1/best_modified_yolov8_v3.bin 时自动选用）
  --yolo-roi-filter        开启 YOLO 顶部/极端异常框过滤（enable_roi_geo_filter=true）
  --finish-heading <JSON>  启用独立终点对正实验（需要已验证地面标定）
  --skip-chassis           底盘已在其他终端运行时跳过底盘启动
  -h, --help               显示帮助

空格切换启动/暂停，Ctrl+C 退出。计时器请在另一个终端运行
  source /opt/tros/humble/setup.bash
  python3 src/scripts/lap_timer.py --label <本轮标签>
EOF
}

while (($#)); do
  case "$1" in
    --params)
      (($# >= 2)) || { echo "--params 需要路径" >&2; exit 2; }
      PARAMS="$2"; shift 2 ;;
    --map)
      (($# >= 2)) || { echo "--map 需要 ring 或 fork" >&2; exit 2; }
      case "$2" in
        ring|roundabout|环岛) MAP="ring" ;;
        fork|crossroad|岔路) MAP="fork" ;;
        *) echo "不支持的地图：$2（可选 ring 或 fork）" >&2; exit 2 ;;
      esac
      shift 2 ;;
    --yolo) START_YOLO=true; shift ;;
    --yolo-model)
      (($# >= 2)) || { echo "--yolo-model 需要 .bin 模型路径" >&2; exit 2; }
      YOLO_MODEL="$2"; shift 2 ;;
    --yolo-roi-filter) YOLO_ROI_FILTER=true; shift ;;
    --finish-heading)
      (($# >= 2)) || { echo "--finish-heading 需要标定 JSON 路径" >&2; exit 2; }
      FINISH_HEADING_CALIBRATION="$2"; shift 2 ;;
    --skip-chassis) SKIP_CHASSIS=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

resolve_params() {
  local requested="$1" candidate matches=() file
  if [[ -f "$requested" ]]; then
    (cd "$(dirname "$requested")" && printf '%s/%s\n' "$PWD" "$(basename "$requested")")
    return 0
  fi
  candidate="$CONFIG_DIR/$requested"
  if [[ -f "$candidate" ]]; then printf '%s\n' "$candidate"; return 0; fi
  while IFS= read -r -d '' file; do
    [[ "$(basename "$file")" == "$requested" ]] && matches+=("$file")
  done < <(find "$CONFIG_DIR" -type f \( -name '*.yaml' -o -name '*.yml' \) -print0)
  if ((${#matches[@]} == 1)); then printf '%s\n' "${matches[0]}"; return 0; fi
  if ((${#matches[@]} > 1)); then
    printf '参数文件名不唯一，请使用相对路径：\n' >&2
    printf '  %s\n' "${matches[@]}" >&2
  fi
  return 1
}

parameter_usage_key() {
  local param_path="$1"
  if [[ "$param_path" == "$CONFIG_DIR"/* ]]; then
    printf '%s\n' "${param_path#"$CONFIG_DIR"/}"
  else
    printf 'external:%s\n' "$param_path"
  fi
}

parameter_usage_count() {
  local key="$1"
  if [[ -f "$PARAM_USAGE_FILE" ]]; then
    awk -F '\t' -v key="$key" '$2 == key { count=$1 } END { print count+0 }' \
      "$PARAM_USAGE_FILE"
  else
    printf '0\n'
  fi
}

record_parameter_use() {
  local param_path="$1" key tmp
  key="$(parameter_usage_key "$param_path")"
  mkdir -p "$(dirname "$PARAM_USAGE_FILE")"
  [[ -f "$PARAM_USAGE_FILE" ]] || : > "$PARAM_USAGE_FILE"
  tmp="${PARAM_USAGE_FILE}.tmp.$$"
  awk -F '\t' -v key="$key" '
    BEGIN { found=0 }
    $2 == key { $1=$1+1; found=1 }
    { print $1 "\t" $2 }
    END { if (!found) print "1\t" key }
  ' "$PARAM_USAGE_FILE" > "$tmp"
  mv "$tmp" "$PARAM_USAGE_FILE"
}

# Prevent a stale toggle node or manual publisher from fighting this launcher's key node.
assert_start_topic_unclaimed() {
  local info publisher_count toggle_pids
  # ROS graph discovery may lag briefly, while a stale executable on this
  # machine is visible immediately. Catch both cases before launching a new
  # keyboard toggle publisher.
  if command -v pgrep >/dev/null 2>&1; then
    toggle_pids="$(pgrep -x start_toggle_node || true)"
    if [[ -n "$toggle_pids" ]]; then
      echo "检测到已有 start_toggle_node 进程（PID: ${toggle_pids//$'\n'/, }），不会再启动第二个。" >&2
      echo "请先在旧启动终端按 Ctrl+C 退出，再重新运行本启动器。" >&2
      ps -o pid,ppid,etime,args -p "$(printf '%s' "$toggle_pids" | paste -sd, -)" >&2 || true
      return 1
    fi
  fi

  info="$(timeout -k 1 4 ros2 topic info --verbose "$START_CMD_TOPIC" 2>&1 || true)"
  publisher_count="$(printf '%s\n' "$info" | awk -F: '/Publisher count:/ { gsub(/[^0-9]/, "", $2); print $2; exit }')"
  if [[ "$publisher_count" =~ ^[0-9]+$ ]] && ((publisher_count > 0)); then
    echo "检测到 $START_CMD_TOPIC 已有 $publisher_count 个发布者，拒绝启动第二个启停源。" >&2
    echo "请退出旧的 start_toggle_node 或 ros2 topic pub，再重新运行。发布者信息：" >&2
    printf '%s\n' "$info" >&2
    return 1
  fi
}

choose_map() {
  local selected
  echo "选择赛道地图：" >&2
  echo "  1) 环岛图" >&2
  echo "  2) 岔路图" >&2
  while true; do
    read -r -p "地图 [1/2]: " selected
    case "$selected" in
      1|ring|roundabout|环岛) printf 'ring\n'; return 0 ;;
      2|fork|crossroad|岔路) printf 'fork\n'; return 0 ;;
      *) echo "请输入 1（环岛图）或 2（岔路图）。" >&2 ;;
    esac
  done
}

is_preset_for_map() {
  local map="$1" relative_path="$2" base
  base="$(basename "$relative_path")"

  # These were reported to press lines and remain hidden from either menu.
  case "$base" in
    HS70_070_y002_curve022.yaml|L_vision_050.yaml|M_vision_055.yaml|\
    RB_S070_070_R040_P050_Y005_ROI058.yaml)
      return 1 ;;
  esac

  case "$map:$base" in
    # Ring presets from the ring-map result table. The K255 HOLD variants are
    # explicitly assigned to the ring map based on the user's successful runs,
    # despite their historical RB_FORK filename prefix.
    ring:line_follower_params.yaml|ring:EXP_*.yaml|ring:C11_058_curve025_y0020_hold10.yaml|\
    ring:HS65_065_y002_curve022.yaml|ring:HS68_068_y002_curve022.yaml|\
    ring:P01_055_shared_strengths.yaml|ring:P02_058_shared_strengths.yaml|\
    ring:RB_C11_058_reverse*.yaml|ring:RB_HS70_070_reverse*.yaml|\
    ring:RB_FORK85_*K255_Y170_Y020*.yaml)
      return 0 ;;

    # Fork-map presets: S is assigned here per the latest field report.
    fork:line_follower_params.yaml|fork:EXP_*.yaml|fork:S_060_thr005.yaml|fork:RB_S*.yaml|\
    fork:RB_FORK72_*.yaml|fork:RB_FORK74_*.yaml|fork:RB_FORK80_*.yaml|\
    fork:RB_FORK85_*K235*.yaml|fork:RB_FORK85_*K245*.yaml|fork:RB_FORK87_*.yaml)
      return 0 ;;
  esac
  return 1
}

choose_params() {
  local map="$1" files=() file selected
  local relative_path usage_count map_label
  case "$map" in
    ring) map_label="环岛图" ;;
    fork) map_label="岔路图" ;;
    *) echo "内部错误：未知地图类型 $map" >&2; return 2 ;;
  esac

  while IFS= read -r file; do
    relative_path="${file#"$CONFIG_DIR"/}"
    is_preset_for_map "$map" "$relative_path" && files+=("$file")
  done < <(find "$CONFIG_DIR" -type f \( -name '*.yaml' -o -name '*.yml' \) -print | sort)

  ((${#files[@]})) || { echo "$map_label 没有可用 YAML：$CONFIG_DIR" >&2; return 1; }
  echo "$map_label参数：" >&2
  local i=1
  for file in "${files[@]}"; do
    relative_path="${file#"$CONFIG_DIR"/}"
    usage_count="$(parameter_usage_count "$relative_path")"
    if [[ "$relative_path" == parameter_presets/EXP_*.yaml ]]; then
      printf '  %2d) [实验功能] %s  [使用 %s 次]\n' \
        "$i" "$relative_path" "$usage_count" >&2
    elif [[ "$relative_path" == parameter_presets/RB_*reverse*.yaml ||
          "$relative_path" == parameter_presets/RB_S*.yaml ]]; then
      printf '  %2d) [Reverse 制动实验] %s  [使用 %s 次]\n' \
        "$i" "$relative_path" "$usage_count" >&2
    else
      printf '  %2d) %s  [使用 %s 次]\n' \
        "$i" "$relative_path" "$usage_count" >&2
    fi
    ((i+=1))
  done
  while true; do
    read -r -p "编号 [默认 1，原版参数]: " selected
    selected="${selected:-1}"
    if [[ "$selected" =~ ^[0-9]+$ ]] && ((selected >= 1 && selected <= ${#files[@]})); then
      printf '%s\n' "${files[selected-1]}"
      return 0
    fi
    echo "请输入 1 到 ${#files[@]} 之间的编号。" >&2
  done
}
if [[ -n "$PARAMS" ]]; then
  PARAMS="$(resolve_params "$PARAMS")" || { echo "找不到参数 YAML：$PARAMS" >&2; exit 2; }
  if [[ -n "$MAP" ]]; then
    [[ "$PARAMS" == "$CONFIG_DIR"/* ]] || {
      echo "--map 只能校验 config/ 内的参数文件；外部 YAML 请省略 --map。" >&2
      exit 2
    }
    selected_relative="${PARAMS#"$CONFIG_DIR"/}"
    is_preset_for_map "$MAP" "$selected_relative" || {
      echo "参数文件不属于所选地图：$([[ "$MAP" == ring ]] && echo 环岛图 || echo 岔路图)：$selected_relative" >&2
      exit 2
    }
  fi
else
  [[ -n "$MAP" ]] || MAP="$(choose_map)" || exit 2
  PARAMS="$(choose_params "$MAP")" || exit 2
fi

if [[ -n "$FINISH_HEADING_CALIBRATION" ]]; then
  if [[ "$FINISH_HEADING_CALIBRATION" != /* ]]; then
    for candidate in \
      "$LINE_WS/src/line_follower/config/$FINISH_HEADING_CALIBRATION" \
      "$LINE_WS/$FINISH_HEADING_CALIBRATION" \
      "$ROOT/$FINISH_HEADING_CALIBRATION"; do
      if [[ -f "$candidate" ]]; then
        FINISH_HEADING_CALIBRATION="$candidate"
        break
      fi
    done
  fi
  [[ -f "$FINISH_HEADING_CALIBRATION" ]] || {
    echo "找不到终点对正地面标定：$FINISH_HEADING_CALIBRATION" >&2; exit 1;
  }
  [[ -f "$LINE_WS/install/line_follower/share/line_follower/launch/line_follower_finish_heading.launch.py" ]] || {
    echo "当前 LINE_WS 未安装终点对正入口；请先构建独立实验工作区。" >&2; exit 1;
  }
fi

[[ -f "$TROS_SETUP" ]] || { echo "找不到 ROS 环境：$TROS_SETUP" >&2; exit 1; }
[[ -f "$LINE_WS/install/setup.bash" ]] || { echo "请先构建 line_follow_ws" >&2; exit 1; }
[[ -x "$LINE_WS/install/line_follower/lib/line_follower/start_toggle_node" ]] || {
  echo "找不到 start_toggle_node，请先构建 line_follower。" >&2; exit 1;
}
[[ -f "$CAMERA_DIR/camera.py" ]] || { echo "找不到相机脚本：$CAMERA_DIR/camera.py" >&2; exit 1; }
if [[ -n "$FINISH_HEADING_CALIBRATION" ]]; then
  spawn follower "$LINE_WS" ros2 launch line_follower line_follower_finish_heading.launch.py \
    "params_file:=$PARAMS" \
    "calibration_file:=$FINISH_HEADING_CALIBRATION" \
    "enable_finish_heading:=true" \
    "use_yolo:=$START_YOLO"
elif [[ "$START_YOLO" == true ]]; then
  [[ -f "$YOLO_DIR/bpu_yolov8_node.py" ]] || { echo "找不到 YOLO 节点" >&2; exit 1; }
  [[ -f "$LINE_WS/install/line_follower/share/line_follower/launch/line_follower_with_detection.launch.py" ]] || {
    echo "找不到 line_follower_with_detection.launch.py" >&2; exit 1;
  }
else
  [[ -f "$LINE_WS/install/line_follower/share/line_follower/launch/line_follower.launch.py" ]] || {
    echo "找不到 line_follower.launch.py" >&2; exit 1;
  }
fi
if [[ "$SKIP_CHASSIS" == false ]]; then
  [[ -f "$CAR_WS/install/setup.bash" ]] || { echo "请先构建 car_ws" >&2; exit 1; }
fi

set +u
source "$TROS_SETUP" >/dev/null
if [[ "$SKIP_CHASSIS" == false ]]; then source "$CAR_WS/install/setup.bash" >/dev/null; fi
source "$LINE_WS/install/setup.bash" >/dev/null
set -u

if ! assert_start_topic_unclaimed; then
  exit 1
fi

mkdir -p "$LOG_ROOT"
LOG_DIR="$LOG_ROOT/line_following_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

stop_group() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM HUP
  [[ -n "$TAIL_PID" ]] && kill "$TAIL_PID" 2>/dev/null || true
  [[ -n "$TOGGLE_PID" ]] && kill "$TOGGLE_PID" 2>/dev/null || true
  if [[ -f "$ROOT/src/scripts/stop_competition.py" ]] && command -v python3 >/dev/null; then
    python3 "$ROOT/src/scripts/stop_competition.py" --pause-only --timeout 1 >/dev/null 2>&1 || true
  elif command -v ros2 >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
    timeout -k 1 2 ros2 topic pub --once -w 1 \
      /line_follower/start_cmd std_msgs/msg/Bool '{data: false}' >/dev/null 2>&1 || true
    sleep .2
  fi
  local pid
  for pid in "${PIDS[@]}"; do [[ "$pid" == "$CHASSIS_PID" ]] || stop_group "$pid"; done
  sleep .3
  if [[ -f "$ROOT/src/scripts/stop_competition.py" ]] && command -v python3 >/dev/null; then
    python3 "$ROOT/src/scripts/stop_competition.py" --timeout 3 || \
      echo "未能确认里程计停稳；请检查车辆并使用硬件急停。" >&2
  fi
  [[ -n "$CHASSIS_PID" ]] && stop_group "$CHASSIS_PID"
  echo "日志保存在：$LOG_DIR"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

spawn() {
  local name="$1" directory="$2"; shift 2
  (cd "$directory" && exec setsid "$@" >"$LOG_DIR/$name.log" 2>&1) &
  local pid=$!
  PIDS+=("$pid")
  [[ "$name" == chassis ]] && CHASSIS_PID="$pid"
  printf '%s\n' "$pid" >"$LOG_DIR/$name.pid"
  echo "已启动 $name，日志：$LOG_DIR/$name.log"
}

command -v setsid >/dev/null 2>&1 || { echo "缺少 setsid" >&2; exit 1; }
if [[ "$SKIP_CHASSIS" == false ]]; then
  spawn chassis "$CAR_WS" ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
  sleep 3
fi
spawn camera "$CAMERA_DIR" python3 camera.py
sleep 2
if [[ "$START_YOLO" == true ]]; then
  YOLO_ARGS=()
  if [[ -z "$YOLO_MODEL" && -f "$YOLO_DIR/mod1/best_modified_yolov8_v3.bin" ]]; then
    YOLO_MODEL="$YOLO_DIR/mod1/best_modified_yolov8_v3.bin"
  fi
  if [[ -n "$YOLO_MODEL" ]]; then
    if [[ "$YOLO_MODEL" != /* ]]; then
      YOLO_MODEL="$ROOT/$YOLO_MODEL"
    fi
    [[ -f "$YOLO_MODEL" ]] || { echo "找不到 YOLO 模型：$YOLO_MODEL" >&2; exit 1; }
    YOLO_ARGS+=(-p "model_path:=$YOLO_MODEL")
  fi
  if [[ "$YOLO_ROI_FILTER" == true ]]; then
    YOLO_ARGS+=(-p "enable_roi_geo_filter:=true")
  fi
  if ((${#YOLO_ARGS[@]})); then
    spawn yolo "$YOLO_DIR" python3 bpu_yolov8_node.py --ros-args "${YOLO_ARGS[@]}"
  else
    spawn yolo "$YOLO_DIR" python3 bpu_yolov8_node.py
  fi
  sleep 2
  spawn follower "$LINE_WS" ros2 launch line_follower line_follower_with_detection.launch.py "params_file:=$PARAMS"
else
  spawn follower "$LINE_WS" ros2 launch line_follower line_follower.launch.py "params_file:=$PARAMS"
fi
record_parameter_use "$PARAMS"
sleep 2

[[ -z "$MAP" ]] || echo "地图：$([[ "$MAP" == ring ]] && echo 环岛图 || echo 岔路图)"
echo "参数：$PARAMS"
echo "该参数已启动次数：$(parameter_usage_count "$(parameter_usage_key "$PARAMS")")"
if [[ "$START_YOLO" == true ]]; then
  echo "YOLO：已启动（模型：$(basename "${YOLO_MODEL:-best_modified_yolov8.bin}")）"
else
  echo "YOLO：未启动"
fi
if [[ -n "$FINISH_HEADING_CALIBRATION" ]]; then
  echo "终点对正：已启用（第三根黄线之后的独立实验；尚未实车验收）"
fi
echo "开始/暂停：在本终端按空格；退出：Ctrl+C"
tail -n +1 -F "$LOG_DIR/follower.log" &
TAIL_PID=$!
"$LINE_WS/install/line_follower/lib/line_follower/start_toggle_node" <&0 &
TOGGLE_PID=$!
wait "$TOGGLE_PID"
