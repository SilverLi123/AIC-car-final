#!/usr/bin/env bash
# Incrementally sync local parameter presets and their launcher to the RDK X5.
set -Eeuo pipefail

ROOT="${AIC_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
PRESET_DIR="$ROOT/src/line_follow_ws/src/line_follower/config/parameter_presets"
LAUNCHER="$ROOT/src/scripts/start_line_following.sh"
CAR_HOST="${CAR_HOST:-sunrise@172.20.10.2}"
CAR_ROOT="${CAR_ROOT:-/home/sunrise/aic_car_final}"
REMOTE_PRESETS="$CAR_ROOT/src/line_follow_ws/src/line_follower/config/parameter_presets"

[[ -d "$PRESET_DIR" ]] || { echo "配置目录不存在：$PRESET_DIR" >&2; exit 1; }
[[ -f "$LAUNCHER" ]] || { echo "启动脚本不存在：$LAUNCHER" >&2; exit 1; }
command -v ssh >/dev/null 2>&1 || { echo "找不到 ssh" >&2; exit 1; }
command -v rsync >/dev/null 2>&1 || { echo "找不到 rsync" >&2; exit 1; }

SSH_OPTIONS=(-o BatchMode=yes -o ConnectTimeout=5)
ssh "${SSH_OPTIONS[@]}" "$CAR_HOST" "mkdir -p '$REMOTE_PRESETS' '$CAR_ROOT/src/scripts'"

# No --delete: this only adds or updates files and preserves other car-side data.
rsync -az --include='*/' --include='*.yaml' --include='*.yml' --exclude='*' \
  "$PRESET_DIR/" "$CAR_HOST:$REMOTE_PRESETS/"
rsync -az "$LAUNCHER" "$CAR_HOST:$CAR_ROOT/src/scripts/"

echo "参数配置和启动菜单已同步到 $CAR_HOST:$CAR_ROOT"
