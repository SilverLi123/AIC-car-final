#!/usr/bin/env bash
# Keep parameter YAMLs and the launcher synced while this Mac user session is active.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SYNC_SCRIPT="$ROOT/src/scripts/sync_parameter_presets.sh"
STATE_DIR="$HOME/Library/Application Support/AICCar"
PID_FILE="$STATE_DIR/parameter-sync.pid"
LOG_FILE="$HOME/Library/Logs/aic-car-parameter-sync.log"
INTERVAL_SECONDS=30

mkdir -p "$STATE_DIR" "$HOME/Library/Logs"

case "${1:-start}" in
  start)
    if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      echo "参数自动同步已在运行，PID $(cat "$PID_FILE")"
      exit 0
    fi
    nohup /bin/bash "$0" run >>"$LOG_FILE" 2>&1 < /dev/null &
    echo "$!" > "$PID_FILE"
    echo "参数自动同步已启动，每 ${INTERVAL_SECONDS} 秒检查一次。PID $(cat "$PID_FILE")"
    ;;
  stop)
    if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      kill "$(cat "$PID_FILE")"
      rm -f "$PID_FILE"
      echo "参数自动同步已停止。"
    else
      rm -f "$PID_FILE"
      echo "参数自动同步当前未运行。"
    fi
    ;;
  status)
    if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
      echo "参数自动同步正在运行，PID $(cat "$PID_FILE")"
    else
      rm -f "$PID_FILE"
      echo "参数自动同步当前未运行。"
      exit 1
    fi
    ;;
  run)
    echo "$$" > "$PID_FILE"
    trap 'rm -f "$PID_FILE"' EXIT INT TERM
    while true; do
      /bin/bash "$SYNC_SCRIPT" || true
      sleep "$INTERVAL_SECONDS"
    done
    ;;
  *)
    echo "用法：$0 [start|stop|status]" >&2
    exit 2
    ;;
esac
