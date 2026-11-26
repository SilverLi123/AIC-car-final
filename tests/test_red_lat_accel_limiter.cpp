#include <cassert>
#include <cmath>
#include <iostream>

#include "line_follower/red_lat_accel_limiter.hpp"

using line_follower::RedLatAccelConfig;
using line_follower::compute_effective_speed;
using line_follower::compute_red_avoid_slew_rate;
using line_follower::compute_red_avoid_yaw_cap;

namespace
{

void test_legacy_disabled_preserves_static_cap()
{
  RedLatAccelConfig cfg;
  cfg.max_yaw_rate = 1.95;
  cfg.red_avoid_max_yaw_rate = 1.25;
  cfg.max_lat_accel = 0.0;
  cfg.highspeed_slew_ref_speed = 0.0;

  assert(std::abs(compute_red_avoid_yaw_cap(cfg, 0.65) - 1.25) < 1e-12);
  assert(std::abs(compute_red_avoid_slew_rate(4.4, cfg, 0.65) - 4.4) < 1e-12);
  std::cout << "[PASS] test_legacy_disabled_preserves_static_cap\n";
}

void test_highspeed_entry_with_deceleration_lag()
{
  RedLatAccelConfig cfg;
  cfg.max_yaw_rate = 2.05;
  cfg.red_avoid_max_yaw_rate = 1.12;
  cfg.max_lat_accel = 0.34;

  const double v_eff = compute_effective_speed(0.26, 0.60, true);
  assert(std::abs(v_eff - 0.60) < 1e-12);

  const double yaw_cap = compute_red_avoid_yaw_cap(cfg, v_eff);
  assert(std::abs(yaw_cap - (0.34 / 0.60)) < 1e-9);
  std::cout << "[PASS] test_highspeed_entry_with_deceleration_lag\n";
}

void test_mid_deceleration_relaxes_yaw_cap()
{
  RedLatAccelConfig cfg;
  cfg.max_yaw_rate = 2.05;
  cfg.red_avoid_max_yaw_rate = 1.12;
  cfg.max_lat_accel = 0.34;

  const double yaw_cap = compute_red_avoid_yaw_cap(cfg, 0.40);
  assert(std::abs(yaw_cap - 0.85) < 1e-9);
  std::cout << "[PASS] test_mid_deceleration_relaxes_yaw_cap\n";
}

void test_slow_tight_bend_keeps_full_authority()
{
  RedLatAccelConfig cfg;
  cfg.max_yaw_rate = 2.05;
  cfg.red_avoid_max_yaw_rate = 1.12;
  cfg.max_lat_accel = 0.34;

  const double yaw_cap = compute_red_avoid_yaw_cap(cfg, 0.22);
  assert(std::abs(yaw_cap - 1.12) < 1e-12);
  std::cout << "[PASS] test_slow_tight_bend_keeps_full_authority\n";
}

void test_stale_odom_falls_back_to_cmd_speed()
{
  const double v_eff = compute_effective_speed(0.26, 0.68, false);
  assert(std::abs(v_eff - 0.26) < 1e-12);
  std::cout << "[PASS] test_stale_odom_falls_back_to_cmd_speed\n";
}

void test_speed_adaptive_slew_scaling_and_floor()
{
  RedLatAccelConfig cfg;
  cfg.highspeed_slew_ref_speed = 0.30;
  cfg.min_slew_scale = 0.45;

  assert(std::abs(compute_red_avoid_slew_rate(3.4, cfg, 0.60) - 1.70) < 1e-9);
  assert(std::abs(compute_red_avoid_slew_rate(3.4, cfg, 0.25) - 3.40) < 1e-9);
  assert(std::abs(compute_red_avoid_slew_rate(3.4, cfg, 1.20) - 1.53) < 1e-9);
  std::cout << "[PASS] test_speed_adaptive_slew_scaling_and_floor\n";
}

}  // namespace

int main()
{
  test_legacy_disabled_preserves_static_cap();
  test_highspeed_entry_with_deceleration_lag();
  test_mid_deceleration_relaxes_yaw_cap();
  test_slow_tight_bend_keeps_full_authority();
  test_stale_odom_falls_back_to_cmd_speed();
  test_speed_adaptive_slew_scaling_and_floor();
  std::cout << "ALL 6 RED LAT-ACCEL LIMITER TESTS PASSED.\n";
  return 0;
}
