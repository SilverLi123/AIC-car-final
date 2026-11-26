#!/bin/bash
set -eo pipefail
TOP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo ">>> [1/2] 编译环岛地图工作区 (ws_rdkx5_ring) ..."
bash "$TOP/ws_rdkx5_ring/build.sh"

echo ">>> [2/2] 编译岔路地图工作区 (ws_rdkx5_fork) ..."
source /opt/tros/humble/setup.bash
if [ -f "$TOP/ws_rdkx5_ring/src/car_ws/install/setup.bash" ]; then
  source "$TOP/ws_rdkx5_ring/src/car_ws/install/setup.bash"
fi
cd "$TOP/ws_rdkx5_fork/src/car_ws"
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd "$TOP/ws_rdkx5_fork/src/line_follow_ws"
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "========================================================"
echo " 双地图全部编译成功！"
echo " 跑【环岛地图】执行: bash $TOP/start_ring.sh"
echo " 跑【岔路地图】执行: bash $TOP/start_fork.sh"
echo "========================================================"
