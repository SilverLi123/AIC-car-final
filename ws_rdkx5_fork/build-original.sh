#!/bin/bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/tros/humble/setup.bash
cd "$ROOT/src/car_ws"
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd "$ROOT/src/line_follow_ws"
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "BUILD_SUCCESS: $ROOT"
