#!/usr/bin/env bash
# Pull a clean source/config snapshot from the onboard aic_car_v2_20261126 workspace.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${1:-sunrise@172.20.10.3}"
REMOTE_DIR="${2:-/home/sunrise/aic_car_v2_20261126}"
DEST_DIR="${3:-$ROOT/onboard_v2_snapshot}"

mkdir -p "$DEST_DIR"
echo "Syncing $HOST:$REMOTE_DIR/src/ -> $DEST_DIR/src/ (excluding build/install/bags/backups)..."
rsync -avz --delete \
  --exclude='**/build/' \
  --exclude='**/install/' \
  --exclude='**/log/' \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='*.bak_*' \
  --exclude='*.db3' \
  --exclude='recorded_bags/' \
  "$HOST:$REMOTE_DIR/src/" "$DEST_DIR/src/"

echo "Snapshot updated at: $DEST_DIR/src/"
