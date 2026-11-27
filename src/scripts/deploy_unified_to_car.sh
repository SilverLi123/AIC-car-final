#!/usr/bin/env bash
# Backward-compatible wrapper delegating to deploy_to_car.sh.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/deploy_to_car.sh" "$@"
