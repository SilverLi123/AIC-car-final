#!/usr/bin/env bash
# Unified AIC competition launcher: select map, YAML preset, YOLO model (.bin),
# and optional synchronized ros2 bag recording (with clean SIGINT flush on exit).
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
BAG_DIR="${BAG_DIR:-$ROOT/recorded_bags}"
START_CMD_TOPIC="/line_follower/start_cmd"

PARAMS=""
MAP=""
YOLO_MODE="auto"       # auto | on | off
YOLO_MODEL=""
YOLO_ROI_FILTER=false
RECORD_MODE="auto"     # auto | on | off
RECORD_TAG=""
RECORD_NOTE=""
RECORD_DEBUG_IMAGE=false
FINISH_HEADING_CALIBRATION=""
SKIP_CHASSIS=false
LIST_MODELS_ONLY=false
LIST_PRESETS_MAP=""
LIST_PRESETS_ONLY=false

PIDS=()
CHASSIS_PID=""
RECORD_PID=""
TOGGLE_PID=""
TAIL_PID=""
LOG_DIR=""

usage() {
  cat <<'EOF'
用法：
  bash src/scripts/start_line_following.sh [选项]

不带参数直接运行时，交互菜单依次选择：
  1) 赛道地图（环岛图 ring / 岔路图 fork）
  2) 参数预设 YAML（按 OPT4 / OPT3 / 默认排序，显示使用次数）
  3) 是否启用 YOLO 及具体 .bin 模型文件（显示文件大小与 MD5 前8位）
  4) 是否同步录包（自动绑定地图、参数与模型元数据）

常用选项：
  --map ring|fork          选择赛道地图（与 --params 同用时自动校验/推断）
  --params <路径或文件名>  直接指定巡线参数 YAML
  --yolo                   启用 YOLO 数字识别节点
  --no-yolo                禁用 YOLO，仅启动 C++ 巡线避障链路
  --yolo-model <名称/路径> 指定 YOLO .bin 模型（可传文件名或绝对/相对路径，自动启用 YOLO）
  --yolo-roi-filter        开启 YOLO 顶部/极端异常框过滤（enable_roi_geo_filter=true）
  --record                 同步后台录包（自动用预设名作为 tag）
  --record-tag <标签>      同步后台录包并指定 tag（例如 R3_S060 或 F3_S044）
  --no-record              不录包，跳过交互询问
  --debug-image            录包时附带 /line_follower/debug_image（体积较大）
  --bag-dir <目录>         指定录包输出目录（默认 <repo>/recorded_bags）
  --note <文本>            为本次录包附加备注（写入 run_meta.json）
  --finish-heading <JSON>  启用独立终点对正实验（需要已验证地面标定）
  --skip-chassis           底盘已在其他终端运行时跳过底盘启动
  --list-models            列出自动发现的 YOLO .bin 模型并退出
  --list-presets [地图]    列出可用的参数预设（可选 ring / fork）并退出
  -h, --help               显示帮助

空格切换启动/暂停，Ctrl+C 退出（若启用录包会先 SIGINT 等待索引刷新完毕）。
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
    --yolo) YOLO_MODE="on"; shift ;;
    --no-yolo) YOLO_MODE="off"; shift ;;
    --yolo-model)
      (($# >= 2)) || { echo "--yolo-model 需要 .bin 模型路径或文件名" >&2; exit 2; }
      YOLO_MODEL="$2"
      [[ "$YOLO_MODE" == "off" ]] || YOLO_MODE="on"
      shift 2 ;;
    --yolo-roi-filter)
      YOLO_ROI_FILTER=true
      [[ "$YOLO_MODE" == "off" ]] || YOLO_MODE="on"
      shift ;;
    --record)
      RECORD_MODE="on"; shift ;;
    --record-tag)
      (($# >= 2)) || { echo "--record-tag 需要标签" >&2; exit 2; }
      RECORD_MODE="on"; RECORD_TAG="$2"; shift 2 ;;
    --no-record)
      RECORD_MODE="off"; shift ;;
    --debug-image)
      RECORD_DEBUG_IMAGE=true; shift ;;
    --bag-dir)
      (($# >= 2)) || { echo "--bag-dir 需要目录" >&2; exit 2; }
      BAG_DIR="$2"; shift 2 ;;
    --note)
      (($# >= 2)) || { echo "--note 需要文本" >&2; exit 2; }
      RECORD_NOTE="$2"; shift 2 ;;
    --finish-heading)
      (($# >= 2)) || { echo "--finish-heading 需要标定 JSON 路径" >&2; exit 2; }
      FINISH_HEADING_CALIBRATION="$2"; shift 2 ;;
    --skip-chassis) SKIP_CHASSIS=true; shift ;;
    --list-models) LIST_MODELS_ONLY=true; shift ;;
    --list-presets)
      LIST_PRESETS_ONLY=true
      if (($# >= 2)) && [[ "$2" != --* ]]; then
        case "$2" in
          ring|roundabout|环岛) LIST_PRESETS_MAP="ring" ;;
          fork|crossroad|岔路) LIST_PRESETS_MAP="fork" ;;
          *) echo "不支持的地图：$2（可选 ring 或 fork）" >&2; exit 2 ;;
        esac
        shift 2
      else
        shift
      fi
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

abs_file_path() {
  local p="$1"
  if [[ -f "$p" ]]; then
    printf '%s/%s\n' "$(cd "$(dirname "$p")" && pwd -P)" "$(basename "$p")"
  else
    printf '%s\n' "$p"
  fi
}

file_size_mb() {
  local p="$1" bytes
  if command -v wc >/dev/null 2>&1; then
    bytes="$(wc -c < "$p" | tr -d '[:space:]')"
    awk -v b="$bytes" 'BEGIN { printf "%.1f", b / (1024.0 * 1024.0) }'
  else
    printf '?.'
  fi
}

file_md5_short() {
  local p="$1" sum=""
  if command -v md5sum >/dev/null 2>&1; then
    sum="$(md5sum "$p" | awk '{print $1}')"
  elif command -v md5 >/dev/null 2>&1; then
    sum="$(md5 -q "$p")"
  elif command -v python3 >/dev/null 2>&1; then
    sum="$(python3 -c 'import hashlib,sys;print(hashlib.md5(open(sys.argv[1],"rb").read()).hexdigest())' "$p")"
  fi
  printf '%s\n' "${sum:0:8}"
}

discover_yolo_models() {
  local candidates=() seen_paths=() file real_p dup
  local search_globs=(
    "$YOLO_DIR/mod1"
    "$YOLO_DIR"
    "$ROOT/models"
    "$HOME"
  )
  local dir
  for dir in "${search_globs[@]}"; do
    [[ -d "$dir" ]] || continue
    while IFS= read -r file; do
      [[ -f "$file" ]] || continue
      if [[ "$dir" == "$HOME" ]]; then
        case "$(basename "$file")" in
          best_modified_yolov8*.bin|aic_*.bin) ;;
          *) continue ;;
        esac
      fi
      real_p="$(abs_file_path "$file")"
      dup=false
      if ((${#seen_paths[@]})); then
        local existing
        for existing in "${seen_paths[@]}"; do
          if [[ "$existing" == "$real_p" ]]; then
            dup=true
            break
          fi
        done
      fi
      if [[ "$dup" == false ]]; then
        seen_paths+=("$real_p")
        candidates+=("$real_p")
      fi
    done < <(find "$dir" -maxdepth 1 -type f -name '*.bin' 2>/dev/null | sort)
  done

  if ((${#candidates[@]})); then
    printf '%s\n' "${candidates[@]}"
  fi
}

resolve_yolo_model() {
  local requested="$1" matches=() model
  if [[ -f "$requested" ]]; then
    abs_file_path "$requested"
    return 0
  fi
  if [[ -f "$ROOT/$requested" ]]; then
    abs_file_path "$ROOT/$requested"
    return 0
  fi
  if [[ -f "$YOLO_DIR/mod1/$requested" ]]; then
    abs_file_path "$YOLO_DIR/mod1/$requested"
    return 0
  fi
  if [[ -f "$YOLO_DIR/$requested" ]]; then
    abs_file_path "$YOLO_DIR/$requested"
    return 0
  fi
  while IFS= read -r model; do
    [[ -n "$model" && "$(basename "$model")" == "$requested" ]] && matches+=("$model")
  done < <(discover_yolo_models)
  if ((${#matches[@]} == 1)); then
    printf '%s\n' "${matches[0]}"
    return 0
  fi
  if ((${#matches[@]} > 1)); then
    echo "YOLO 模型文件名不唯一，请指定完整路径：" >&2
    printf '  %s\n' "${matches[@]}" >&2
  fi
  return 1
}

choose_yolo_model() {
  local models=() m idx=1 selected custom_path sz md5 dir_hint
  while IFS= read -r m; do
    [[ -n "$m" ]] && models+=("$m")
  done < <(discover_yolo_models)

  if ((${#models[@]} == 0)); then
    echo "未自动发现 .bin 模型，请输入模型路径：" >&2
    read -r -p "YOLO 模型路径: " custom_path
    [[ -n "$custom_path" ]] || return 1
    resolve_yolo_model "$custom_path"
    return $?
  fi

  echo "选择 YOLO .bin 模型：" >&2
  for m in "${models[@]}"; do
    sz="$(file_size_mb "$m")"
    md5="$(file_md5_short "$m")"
    dir_hint="$(basename "$(dirname "$m")")"
    printf '  %2d) %-32s | %5s MB | md5:%s | (%s)\n' \
      "$idx" "$(basename "$m")" "$sz" "${md5:--}" "$dir_hint" >&2
    ((idx += 1))
  done
  echo "   m) 手动输入其他 .bin 路径" >&2

  while true; do
    read -r -p "模型编号 [默认 1]: " selected
    selected="${selected:-1}"
    if [[ "$selected" == "m" || "$selected" == "M" ]]; then
      read -r -p "输入 .bin 路径: " custom_path
      if resolve_yolo_model "$custom_path"; then
        return 0
      fi
      echo "找不到模型：$custom_path" >&2
      continue
    fi
    if [[ "$selected" =~ ^[0-9]+$ ]] && ((selected >= 1 && selected <= ${#models[@]})); then
      printf '%s\n' "${models[selected-1]}"
      return 0
    fi
    echo "请输入 1 到 ${#models[@]} 之间的编号，或输入 m。" >&2
  done
}

resolve_params() {
  local requested="$1" candidate matches=() file
  if [[ -f "$requested" ]]; then
    abs_file_path "$requested"
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

assert_start_topic_unclaimed() {
  local info publisher_count toggle_pids
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
  echo "  1) 环岛图 (ring)" >&2
  echo "  2) 岔路图 (fork)" >&2
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

  case "$base" in
    HS70_070_y002_curve022.yaml|L_vision_050.yaml|M_vision_055.yaml|\
    RB_S070_070_R040_P050_Y005_ROI058.yaml)
      return 1 ;;
  esac

  case "$map:$base" in
    ring:line_follower_params.yaml|ring:EXP_*.yaml|ring:YOLO_RING*.yaml|\
    ring:C11_058_curve025_y0020_hold10.yaml|\
    ring:HS65_065_y002_curve022.yaml|ring:HS68_068_y002_curve022.yaml|\
    ring:P01_055_shared_strengths.yaml|ring:P02_058_shared_strengths.yaml|\
    ring:RB_C11_058_reverse*.yaml|ring:RB_HS70_070_reverse*.yaml|\
    ring:RB_FORK85_*K255_Y170_Y020*.yaml)
      return 0 ;;

    fork:line_follower_params.yaml|fork:EXP_*.yaml|fork:YOLO_FORK*.yaml|\
    fork:S_060_thr005.yaml|fork:RB_S*.yaml|\
    fork:RB_FORK72_*.yaml|fork:RB_FORK74_*.yaml|fork:RB_FORK80_*.yaml|\
    fork:RB_FORK85_*K235*.yaml|fork:RB_FORK85_*K245*.yaml|fork:RB_FORK87_*.yaml)
      return 0 ;;
  esac
  return 1
}

preset_priority_rank() {
  local base
  base="$(basename "$1")"
  case "$base" in
    YOLO_RING_OPT4_*|YOLO_FORK_OPT4_*) printf '10\n' ;;
    YOLO_RING_OPT3_*|YOLO_FORK_OPT3_*) printf '20\n' ;;
    YOLO_*) printf '30\n' ;;
    line_follower_params.yaml) printf '40\n' ;;
    EXP_*) printf '50\n' ;;
    *) printf '60\n' ;;
  esac
}

preset_tag_label() {
  local base
  base="$(basename "$1")"
  case "$base" in
    YOLO_RING_OPT4_*) printf '[OPT4 环岛] ' ;;
    YOLO_FORK_OPT4_*) printf '[OPT4 岔路] ' ;;
    YOLO_RING_OPT3_*) printf '[OPT3 基线] ' ;;
    line_follower_params.yaml) printf '[原版默认] ' ;;
    EXP_*) printf '[实验功能] ' ;;
    RB_*reverse*.yaml|RB_S*.yaml) printf '[Reverse]   ' ;;
    *) printf '' ;;
  esac
}

collect_map_presets() {
  local map="$1" file rel rank
  while IFS= read -r file; do
    rel="${file#"$CONFIG_DIR"/}"
    if is_preset_for_map "$map" "$rel"; then
      rank="$(preset_priority_rank "$rel")"
      printf '%s\t%s\n' "$rank" "$file"
    fi
  done < <(find "$CONFIG_DIR" -type f \( -name '*.yaml' -o -name '*.yml' \) -print) | \
    LC_ALL=C sort -t $'\t' -k1,1n -k2,2 | cut -f2-
}

choose_params() {
  local map="$1" files=() file selected
  local relative_path usage_count map_label tag_label display_name
  case "$map" in
    ring) map_label="环岛图" ;;
    fork) map_label="岔路图" ;;
    *) echo "内部错误：未知地图类型 $map" >&2; return 2 ;;
  esac

  while IFS= read -r file; do
    [[ -n "$file" ]] && files+=("$file")
  done < <(collect_map_presets "$map")

  ((${#files[@]})) || { echo "$map_label 没有可用 YAML：$CONFIG_DIR" >&2; return 1; }
  echo "$map_label参数预设：" >&2
  local i=1
  for file in "${files[@]}"; do
    relative_path="${file#"$CONFIG_DIR"/}"
    usage_count="$(parameter_usage_count "$relative_path")"
    tag_label="$(preset_tag_label "$relative_path")"
    display_name="$(basename "$file")"
    printf '  %2d) %s%-42s [使用 %s 次]\n' \
      "$i" "$tag_label" "$display_name" "$usage_count" >&2
    ((i += 1))
  done
  while true; do
    read -r -p "编号 [默认 1]: " selected
    selected="${selected:-1}"
    if [[ "$selected" =~ ^[0-9]+$ ]] && ((selected >= 1 && selected <= ${#files[@]})); then
      printf '%s\n' "${files[selected-1]}"
      return 0
    fi
    echo "请输入 1 到 ${#files[@]} 之间的编号。" >&2
  done
}

# Non-executing query flags
if [[ "$LIST_MODELS_ONLY" == true ]]; then
  models=()
  while IFS= read -r m; do
    [[ -n "$m" ]] && models+=("$m")
  done < <(discover_yolo_models)
  if ((${#models[@]} == 0)); then
    echo "未发现 .bin 模型文件。"
    exit 0
  fi
  echo "已发现 YOLO 模型（共 ${#models[@]} 个）："
  idx=1
  for m in "${models[@]}"; do
    printf '  %2d) %-32s | %5s MB | md5:%s | %s\n' \
      "$idx" "$(basename "$m")" "$(file_size_mb "$m")" "$(file_md5_short "$m")" "$m"
    ((idx += 1))
  done
  exit 0
fi

if [[ "$LIST_PRESETS_ONLY" == true ]]; then
  maps_to_list=()
  if [[ -n "$LIST_PRESETS_MAP" ]]; then
    maps_to_list=("$LIST_PRESETS_MAP")
  else
    maps_to_list=("ring" "fork")
  fi
  for m_name in "${maps_to_list[@]}"; do
    echo "=== $([[ "$m_name" == ring ]] && echo "环岛图 (ring)" || echo "岔路图 (fork)") ==="
    idx=1
    while IFS= read -r file; do
      [[ -n "$file" ]] || continue
      rel="${file#"$CONFIG_DIR"/}"
      printf '  %2d) %s%-42s (%s)\n' \
        "$idx" "$(preset_tag_label "$rel")" "$(basename "$file")" "$rel"
      ((idx += 1))
    done < <(collect_map_presets "$m_name")
  done
  exit 0
fi

# 1) Resolve map + params
if [[ -n "$PARAMS" ]]; then
  PARAMS="$(resolve_params "$PARAMS")" || { echo "找不到参数 YAML：$PARAMS" >&2; exit 2; }
  if [[ -z "$MAP" ]]; then
    case "$(basename "$PARAMS")" in
      YOLO_RING*) MAP="ring" ;;
      YOLO_FORK*) MAP="fork" ;;
    esac
  fi
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

# 2) Resolve YOLO toggle & model
if [[ "$YOLO_MODE" == "auto" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "是否启用 YOLO 数字识别？[Y/n]: " ans_yolo
    case "$ans_yolo" in
      n|N|no|NO) YOLO_MODE="off" ;;
      *) YOLO_MODE="on" ;;
    esac
  else
    YOLO_MODE="off"
  fi
fi

START_YOLO=false
if [[ "$YOLO_MODE" == "on" ]]; then
  START_YOLO=true
  if [[ -n "$YOLO_MODEL" ]]; then
    YOLO_MODEL="$(resolve_yolo_model "$YOLO_MODEL")" || {
      echo "找不到 YOLO 模型：$YOLO_MODEL" >&2
      exit 1
    }
  elif [[ -t 0 ]]; then
    YOLO_MODEL="$(choose_yolo_model)" || {
      echo "YOLO 模型选择失败。" >&2
      exit 1
    }
  else
    YOLO_MODEL="$(resolve_yolo_model "best_modified_yolov8.bin")" || true
  fi
fi

# 3) Resolve integrated bag recording
if [[ "$RECORD_MODE" == "auto" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "是否同步后台录包？[y/N]: " ans_rec
    case "$ans_rec" in
      y|Y|yes|YES)
        RECORD_MODE="on"
        default_tag="$(basename "$PARAMS" .yaml)"
        read -r -p "录包标签 [默认 ${default_tag}]: " input_tag
        RECORD_TAG="${input_tag:-$default_tag}"
        read -r -p "是否同时录制 debug_image（体积显著增大）？[y/N]: " ans_dbg
        case "$ans_dbg" in
          y|Y|yes|YES) RECORD_DEBUG_IMAGE=true ;;
        esac
        ;;
      *) RECORD_MODE="off" ;;
    esac
  else
    RECORD_MODE="off"
  fi
fi

if [[ "$RECORD_MODE" == "on" && -z "$RECORD_TAG" ]]; then
  RECORD_TAG="$(basename "$PARAMS" .yaml)"
fi

# 4) Pre-flight checks (NEVER spawn inside pre-flight)
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
if [[ "$START_YOLO" == true ]]; then
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
if [[ "$RECORD_MODE" == "on" ]]; then
  [[ -f "$ROOT/src/scripts/record_bag.sh" ]] || {
    echo "找不到录包脚本：$ROOT/src/scripts/record_bag.sh" >&2; exit 1;
  }
fi

set +u
# shellcheck disable=SC1090
source "$TROS_SETUP" >/dev/null
if [[ "$SKIP_CHASSIS" == false ]]; then
  # shellcheck disable=SC1090
  source "$CAR_WS/install/setup.bash" >/dev/null
fi
# shellcheck disable=SC1090
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

stop_recorder_gracefully() {
  local pid="$1" waited=0
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    echo ""
    echo "正在通知后台录包退出并刷新索引..."
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    while kill -0 "$pid" 2>/dev/null && ((waited < 60)); do
      sleep 0.1
      ((waited += 1))
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  fi
  if [[ -f "$LOG_DIR/record.log" ]]; then
    grep -E "录包完成|analyze_bag\.py" "$LOG_DIR/record.log" || true
  fi
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

  # Flush rosbag2 metadata via SIGINT BEFORE tearing down ROS nodes
  stop_recorder_gracefully "$RECORD_PID"

  local pid
  if ((${#PIDS[@]})); then
    for pid in "${PIDS[@]}"; do
      [[ "$pid" == "$CHASSIS_PID" || "$pid" == "$RECORD_PID" ]] && continue
      stop_group "$pid"
    done
  fi
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
  [[ "$name" == record ]] && RECORD_PID="$pid"
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
  if [[ -n "$YOLO_MODEL" ]]; then
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
fi

if [[ -n "$FINISH_HEADING_CALIBRATION" ]]; then
  spawn follower "$LINE_WS" ros2 launch line_follower line_follower_finish_heading.launch.py \
    "params_file:=$PARAMS" \
    "calibration_file:=$FINISH_HEADING_CALIBRATION" \
    "enable_finish_heading:=true" \
    "use_yolo:=$START_YOLO"
elif [[ "$START_YOLO" == true ]]; then
  spawn follower "$LINE_WS" ros2 launch line_follower line_follower_with_detection.launch.py "params_file:=$PARAMS"
else
  spawn follower "$LINE_WS" ros2 launch line_follower line_follower.launch.py "params_file:=$PARAMS"
fi

record_parameter_use "$PARAMS"
sleep 2

if [[ "$RECORD_MODE" == "on" ]]; then
  REC_ARGS=(--tag "$RECORD_TAG" --params "$PARAMS" --bag-dir "$BAG_DIR")
  [[ -z "$MAP" ]] || REC_ARGS+=(--map "$MAP")
  [[ -z "$YOLO_MODEL" || "$START_YOLO" != true ]] || REC_ARGS+=(--model "$YOLO_MODEL")
  [[ -z "$RECORD_NOTE" ]] || REC_ARGS+=(--note "$RECORD_NOTE")
  [[ "$RECORD_DEBUG_IMAGE" == true ]] && REC_ARGS+=(--debug-image)
  spawn record "$ROOT" bash "$ROOT/src/scripts/record_bag.sh" "${REC_ARGS[@]}"
  sleep 1
fi

echo "========================================"
[[ -z "$MAP" ]] || echo "地图：$([[ "$MAP" == ring ]] && echo "环岛图 (ring)" || echo "岔路图 (fork)")"
echo "参数：$(basename "$PARAMS")（已使用 $(parameter_usage_count "$(parameter_usage_key "$PARAMS")") 次）"
echo "路径：$PARAMS"
if [[ "$START_YOLO" == true ]]; then
  if [[ -n "$YOLO_MODEL" && -f "$YOLO_MODEL" ]]; then
    echo "YOLO：已启动 | 模型=$(basename "$YOLO_MODEL") ($(file_size_mb "$YOLO_MODEL") MB, md5:$(file_md5_short "$YOLO_MODEL")) | roi_filter=$YOLO_ROI_FILTER"
  else
    echo "YOLO：已启动 | 默认模型 | roi_filter=$YOLO_ROI_FILTER"
  fi
else
  echo "YOLO：未启动"
fi
if [[ "$RECORD_MODE" == "on" ]]; then
  echo "录包：已开启 | tag=$RECORD_TAG | dir=$BAG_DIR | debug_image=$RECORD_DEBUG_IMAGE"
else
  echo "录包：未开启"
fi
if [[ -n "$FINISH_HEADING_CALIBRATION" ]]; then
  echo "终点对正：已启用（第三根黄线之后的独立实验）"
fi
echo "操作：在本终端按 [空格] 启停；按 [Ctrl+C] 安全停车并保存录包"
echo "========================================"

tail -n +1 -F "$LOG_DIR/follower.log" &
TAIL_PID=$!
"$LINE_WS/install/line_follower/lib/line_follower/start_toggle_node" <&0 &
TOGGLE_PID=$!
wait "$TOGGLE_PID"
