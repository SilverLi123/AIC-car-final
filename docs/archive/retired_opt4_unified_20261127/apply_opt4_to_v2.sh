#!/usr/bin/env bash
# Deploy OPT4 lateral-accel & speed-adaptive slew limiter to onboard aic_car_v2_20261126.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${1:-sunrise@172.20.10.3}"
REMOTE_DIR="${2:-/home/sunrise/aic_car_v2_20261126}"

HPP_LOCAL="$ROOT/src/line_follow_ws/src/line_follower/include/line_follower/red_lat_accel_limiter.hpp"
PRESETS_LOCAL="$ROOT/src/line_follow_ws/src/line_follower/config/parameter_presets/ring_opt4_20261127"

[[ -f "$HPP_LOCAL" ]] || { echo "Missing header: $HPP_LOCAL" >&2; exit 2; }
[[ -d "$PRESETS_LOCAL" ]] || { echo "Missing presets dir: $PRESETS_LOCAL" >&2; exit 2; }

echo "=== [1/4] Checking SSH connectivity to $HOST ==="
ssh -o ConnectTimeout=5 "$HOST" "test -d '$REMOTE_DIR/src/line_follow_ws/src/line_follower'"

echo "=== [2/4] Copying OPT4 header and YAML presets to $HOST:$REMOTE_DIR ==="
scp -o ConnectTimeout=5 "$HPP_LOCAL" \
  "$HOST:$REMOTE_DIR/src/line_follow_ws/src/line_follower/include/line_follower/red_lat_accel_limiter.hpp"
scp -o ConnectTimeout=5 "$PRESETS_LOCAL"/YOLO_RING_OPT4_*.yaml \
  "$HOST:$REMOTE_DIR/src/line_follow_ws/src/line_follower/config/parameter_presets/"

echo "=== [3/4] Backing up & idempotently patching remote line_follower_node.cpp ==="
ssh -o ConnectTimeout=5 "$HOST" "REMOTE_DIR='$REMOTE_DIR' python3 -" << 'PYEOF'
import os
from pathlib import Path
import shutil

remote_dir = os.environ["REMOTE_DIR"]
cpp_path = Path(remote_dir) / "src/line_follow_ws/src/line_follower/src/line_follower_node.cpp"
bak_path = cpp_path.with_name("line_follower_node.cpp.bak_pre_opt4")
if not bak_path.exists():
    shutil.copy2(cpp_path, bak_path)

cpp = cpp_path.read_text(encoding="utf-8")

def replace_once(text: str, old: str, new: str, label: str) -> str:
    cnt = text.count(old)
    assert cnt == 1, f"{label}: expected 1 match, got {cnt}"
    return text.replace(old, new, 1)

if '#include "line_follower/red_lat_accel_limiter.hpp"' not in cpp:
    cpp = replace_once(
        cpp,
        '#include "line_follower/line_follower_node.hpp"',
        '#include "line_follower/line_follower_node.hpp"\n#include "line_follower/red_lat_accel_limiter.hpp"',
        "include_header",
    )

if 'red_avoid_max_lat_accel' not in cpp:
    cpp = replace_once(
        cpp,
        '  this->declare_parameter("red_avoid_max_yaw_rate", 9.9);',
        """  this->declare_parameter("red_avoid_max_yaw_rate", 9.9);
  this->declare_parameter("red_avoid_max_lat_accel", 0.0);
  this->declare_parameter("red_avoid_highspeed_slew_ref_speed", 0.0);""",
        "declare_opt4_params",
    )

if "compute_red_avoid_yaw_cap" not in cpp:
    cpp = replace_once(
        cpp,
        "  if (!yellow_stop_now && !tjunction_active_ && soft_red_enabled) {",
        """  line_follower::RedLatAccelConfig red_lat_cfg;
  red_lat_cfg.max_yaw_rate = max_yaw_rate_;
  red_lat_cfg.red_avoid_max_yaw_rate = std::max(
    0.2, this->get_parameter("red_avoid_max_yaw_rate").as_double());
  red_lat_cfg.max_lat_accel = std::max(
    0.0, this->get_parameter("red_avoid_max_lat_accel").as_double());
  red_lat_cfg.highspeed_slew_ref_speed = std::max(
    0.0, this->get_parameter("red_avoid_highspeed_slew_ref_speed").as_double());
  const double red_odom_age =
    odom_received_ ? (now - odom_receive_time_).seconds() : 1e9;
  const bool red_odom_fresh =
    odom_received_ && red_odom_age >= 0.0 && red_odom_age <= 0.25;
  const double red_v_eff = line_follower::compute_effective_speed(
    linear_speed, odom_linear_speed_, red_odom_fresh);

  if (!yellow_stop_now && !tjunction_active_ && soft_red_enabled) {""",
        "prepare_red_lat_cfg",
    )

    cpp = replace_once(
        cpp,
        """      const double red_max_yaw = std::min(
        max_yaw_rate_,
        std::max(0.2, this->get_parameter("red_avoid_max_yaw_rate").as_double()));
      yaw_rate = clamp(yaw_rate, -red_max_yaw, red_max_yaw);""",
        """      const double red_max_yaw =
        line_follower::compute_red_avoid_yaw_cap(red_lat_cfg, red_v_eff);
      yaw_rate = clamp(yaw_rate, -red_max_yaw, red_max_yaw);""",
        "apply_red_yaw_cap",
    )

    cpp = replace_once(
        cpp,
        """    if (in_red_window && red_slew_initialized_) {
      const double dt = clamp((now - red_slew_last_time_).seconds(), 0.005, 0.10);
      const double max_delta = red_yaw_slew * dt;""",
        """    if (in_red_window && red_slew_initialized_) {
      const double dt = clamp((now - red_slew_last_time_).seconds(), 0.005, 0.10);
      const double eff_red_yaw_slew = line_follower::compute_red_avoid_slew_rate(
        red_yaw_slew, red_lat_cfg, red_v_eff);
      const double max_delta = eff_red_yaw_slew * dt;""",
        "apply_red_slew_scale",
    )

cpp_path.write_text(cpp, encoding="utf-8")
print("PATCH_OPT4_OK")
PYEOF

echo "=== [4/4] Building line_follower & verifying OPT4 presets on $HOST ==="
ssh -o ConnectTimeout=5 "$HOST" "bash -s" << EOF
set -e
cd "$REMOTE_DIR/src/line_follow_ws"
source /opt/tros/humble/setup.bash
colcon build --packages-select line_follower
python3 - << 'PYEOF'
import glob, yaml
inst_dir = "$REMOTE_DIR/src/line_follow_ws/install/line_follower/share/line_follower/config/parameter_presets"
files = sorted(glob.glob(f"{inst_dir}/YOLO_RING_OPT4_*.yaml"))
assert files, "No installed YOLO_RING_OPT4 YAML files found"
for p in files:
    d = yaml.safe_load(open(p, encoding="utf-8"))
    lf = d["line_follower_node"]["ros__parameters"]
    assert lf.get("red_avoid_max_lat_accel", 0.0) > 0.0, p
    print(f"VERIFIED: {p} params={len(lf)} lat_accel={lf['red_avoid_max_lat_accel']}")
PYEOF
EOF

echo "Done! Ready to test YOLO_RING_OPT4_S060_LATCAP.yaml on $HOST."
