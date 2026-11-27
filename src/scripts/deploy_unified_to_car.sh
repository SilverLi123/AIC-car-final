#!/usr/bin/env bash
# Sync unified Ring+Fork codebase, scripts, and OPT4 presets to RDK X5 & build line_follower.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${1:-sunrise@172.20.10.3}"
REMOTE_DIR="${2:-/home/sunrise/aic_car_v2_20261126}"

LOCAL_LF="$ROOT/src/line_follow_ws/src/line_follower"
LOCAL_SCRIPTS="$ROOT/src/scripts"
LOCAL_YOLO="$ROOT/src/yolov8/bpu_yolov8_node.py"

[[ -d "$LOCAL_LF" ]] || { echo "Missing local line_follower: $LOCAL_LF" >&2; exit 2; }
[[ -f "$LOCAL_YOLO" ]] || { echo "Missing local YOLO node: $LOCAL_YOLO" >&2; exit 2; }

echo "=== [1/4] Checking SSH connectivity to $HOST ==="
ssh -o ConnectTimeout=5 "$HOST" "test -d '$REMOTE_DIR/src/line_follow_ws/src/line_follower'"

echo "=== [2/4] Backing up current remote line_follower & scripts ==="
ssh -o ConnectTimeout=5 "$HOST" "bash -s" << EOF
set -e
cd "$REMOTE_DIR"
STAMP="\$(date +%Y%m%d_%H%M%S)"
BAK="backup_pre_unified_\${STAMP}.tar.gz"
tar -czf "\$BAK" src/line_follow_ws/src/line_follower src/scripts src/yolov8/bpu_yolov8_node.py
echo "Remote backup created: $REMOTE_DIR/\$BAK"
EOF

echo "=== [3/4] Syncing unified line_follower, scripts, presets, and bpu_yolov8_node.py ==="
if command -v rsync >/dev/null 2>&1 && ssh -o ConnectTimeout=5 "$HOST" "command -v rsync >/dev/null 2>&1"; then
  rsync -avz --delete \
    --exclude '*.bak*' \
    --exclude '__pycache__/' \
    "$LOCAL_LF/" \
    "$HOST:$REMOTE_DIR/src/line_follow_ws/src/line_follower/"
  rsync -avz \
    "$LOCAL_SCRIPTS/" \
    "$HOST:$REMOTE_DIR/src/scripts/"
  rsync -avz \
    "$LOCAL_YOLO" \
    "$HOST:$REMOTE_DIR/src/yolov8/bpu_yolov8_node.py"
else
  scp -r -o ConnectTimeout=5 "$LOCAL_LF" "$HOST:$REMOTE_DIR/src/line_follow_ws/src/"
  scp -o ConnectTimeout=5 "$LOCAL_SCRIPTS"/*.sh "$LOCAL_SCRIPTS"/*.py "$HOST:$REMOTE_DIR/src/scripts/"
  scp -o ConnectTimeout=5 "$LOCAL_YOLO" "$HOST:$REMOTE_DIR/src/yolov8/bpu_yolov8_node.py"
fi

echo "=== [4/4] Building line_follower & verifying 15 OPT4 Ring+Fork YAML presets ==="
ssh -o ConnectTimeout=5 "$HOST" "bash -s" << EOF
set -e
chmod +x "$REMOTE_DIR/src/scripts/"*.sh "$REMOTE_DIR/src/scripts/"*.py
cd "$REMOTE_DIR/src/line_follow_ws"
source /opt/tros/humble/setup.bash
colcon build --packages-select line_follower
python3 - << 'PYEOF'
import glob, yaml
inst_base = "$REMOTE_DIR/src/line_follow_ws/install/line_follower/share/line_follower/config/parameter_presets"
ring_files = sorted(glob.glob(f"{inst_base}/ring_opt4_20261127/YOLO_RING_*.yaml"))
fork_files = sorted(glob.glob(f"{inst_base}/fork_opt4_20261127/YOLO_FORK_*.yaml"))
assert len(ring_files) >= 9, f"Expected >=9 Ring presets, got {len(ring_files)}"
assert len(fork_files) >= 6, f"Expected >=6 Fork presets, got {len(fork_files)}"
for p in ring_files + fork_files:
    d = yaml.safe_load(open(p, encoding="utf-8"))
    lf = d["line_follower_node"]["ros__parameters"]
    dtc = d["detection_turn_controller"]["ros__parameters"]
    assert "target_speed" in lf and "output_topic" in d["cmd_mux_node"]["ros__parameters"]
print(f"VERIFIED: {len(ring_files)} Ring presets + {len(fork_files)} Fork presets installed.")
PYEOF
EOF

echo ""
echo "同步与编译完成！可用下面命令检查车端模型与预设："
echo "  ssh $HOST 'cd $REMOTE_DIR && bash src/scripts/start_line_following.sh --list-models'"
echo "  ssh $HOST 'cd $REMOTE_DIR && bash src/scripts/start_line_following.sh --list-presets'"
