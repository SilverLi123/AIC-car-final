#pragma once

#include <algorithm>
#include <cmath>

namespace line_follower
{

/**
 * Choose a collision-free lateral target x within the contiguous white passable gap
 * [chosen_gl, chosen_gr] observed on the unbridged binary mask next to an obstacle.
 */
inline double compute_passable_gap_target_x(
  double chosen_gl,
  double chosen_gr,
  double normal_x,
  double frame_w,
  bool is_red_zone)
{
  const double rw = std::max(1.0, frame_w);
  const double margin_px = (is_red_zone ? 0.17 : 0.23) * rw;
  const double lo = chosen_gl + margin_px;
  const double hi = chosen_gr - margin_px;
  if (lo <= hi) {
    return std::clamp(normal_x, lo, hi);
  }
  return 0.5 * (chosen_gl + chosen_gr);
}

/**
 * Compute steering yaw rate toward the passable gap target while clamping
 * inward recovery authority so cone avoidance does not pull onto the outer border.
 */
inline double compute_passable_gap_yaw(
  double gap_target_x,
  double frame_w,
  double kp,
  bool is_red_zone,
  bool red_on_left,
  double bottom_ratio,
  double max_yaw_rate)
{
  const double half_w = 0.5 * std::max(1.0, frame_w);
  const double gap_err = std::clamp((gap_target_x - half_w) / half_w, -1.0, 1.0);
  double gap_yaw = -kp * gap_err;
  if (is_red_zone) {
    gap_yaw = std::clamp(gap_yaw, -0.38, 0.38);
  } else {
    const double inward_cap = (bottom_ratio < 0.92) ? 0.62 : 0.95;
    if (red_on_left) {
      gap_yaw = std::clamp(gap_yaw, -1.15, inward_cap);
    } else {
      gap_yaw = std::clamp(gap_yaw, -inward_cap, 1.15);
    }
  }
  return std::clamp(gap_yaw, -max_yaw_rate, max_yaw_rate);
}

}  // namespace line_follower
