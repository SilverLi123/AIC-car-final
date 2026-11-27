#include <cassert>
#include <cmath>
#include <iostream>

#include "line_follower/passable_gap_guidance.hpp"

using line_follower::compute_passable_gap_target_x;
using line_follower::compute_passable_gap_yaw;

namespace
{

void test_wide_gap_keeps_normal_lane_center()
{
  // Wide white lane around cone leaves normal_x strictly inside [lo, hi]
  const double target_x = compute_passable_gap_target_x(
    20.0, 520.0, 310.0, 640.0, false);
  assert(std::abs(target_x - 310.0) < 1e-12);
  std::cout << "[PASS] test_wide_gap_keeps_normal_lane_center\n";
}

void test_narrow_gap_falls_back_to_midpoint()
{
  // When gap width < 2 * margin_px (2 * 0.23 * 640 = 294.4px), target falls back to gap midpoint
  const double target_x = compute_passable_gap_target_x(
    80.0, 260.0, 320.0, 640.0, false);
  assert(std::abs(target_x - 170.0) < 1e-12);
  std::cout << "[PASS] test_narrow_gap_falls_back_to_midpoint\n";
}

void test_red_zone_uses_gentle_yaw_cap_038()
{
  // Red zone on left -> passing right gap -> target shifted far right -> raw yaw negative, clamped at -0.38
  const double yaw = compute_passable_gap_yaw(
    540.0, 640.0, 2.65, true, true, 0.78, 1.65);
  assert(std::abs(yaw - (-0.38)) < 1e-12);
  std::cout << "[PASS] test_red_zone_uses_gentle_yaw_cap_038\n";
}

void test_cone_inward_cap_prevents_over_return_onto_border()
{
  // Cone on left, red_on_left=true, target on left pulls inward (+yaw): capped at +0.62 when bottom < 0.92
  const double yaw_mid = compute_passable_gap_yaw(
    120.0, 640.0, 2.65, false, true, 0.72, 1.65);
  assert(std::abs(yaw_mid - 0.62) < 1e-12);

  // Very close emergency (bottom >= 0.92) relaxes inward recovery cap to +0.95
  const double yaw_close = compute_passable_gap_yaw(
    120.0, 640.0, 2.65, false, true, 0.94, 1.65);
  assert(std::abs(yaw_close - 0.95) < 1e-12);
  std::cout << "[PASS] test_cone_inward_cap_prevents_over_return_onto_border\n";
}

}  // namespace

int main()
{
  test_wide_gap_keeps_normal_lane_center();
  test_narrow_gap_falls_back_to_midpoint();
  test_red_zone_uses_gentle_yaw_cap_038();
  test_cone_inward_cap_prevents_over_return_onto_border();
  std::cout << "ALL 4 PASSABLE GAP GUIDANCE TESTS PASSED.\n";
  return 0;
}
