#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "line_follower/tjunction_merge_detector.hpp"

using line_follower::tjunction::TjunctionConfig;
using line_follower::tjunction::TjunctionDecision;
using line_follower::tjunction::TjunctionMergeController;
using line_follower::tjunction::TjunctionMetrics;
using line_follower::tjunction::analyze_mask;

namespace
{

constexpr int kW = 160;
constexpr int kH = 120;

std::vector<uint8_t> make_straight_mask()
{
  std::vector<uint8_t> m(kW * kH, 0);
  for (int y = 0; y < kH; ++y) {
    for (int x = 50; x < 110; ++x) {
      m[y * kW + x] = 255;
    }
  }
  return m;
}

std::vector<uint8_t> make_perpendicular_wall_mask()
{
  std::vector<uint8_t> m(kW * kH, 0);
  // Dark wall above y=48, white crossbar fills lower half.
  for (int y = 48; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      m[y * kW + x] = 255;
    }
  }
  return m;
}

std::vector<uint8_t> make_tilted_wall_mask(double tilt_rad)
{
  std::vector<uint8_t> m(kW * kH, 0);
  const double slope = std::tan(tilt_rad);
  for (int y = 0; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      const double y_wall = 52.0 + slope * (x - kW * 0.5);
      if (y >= y_wall) {
        m[y * kW + x] = 255;
      }
    }
  }
  return m;
}

void test_straight_or_arch_never_triggers()
{
  TjunctionConfig cfg;
  cfg.enabled = true;
  cfg.confirm_frames = 2;
  TjunctionMergeController ctrl(cfg);
  ctrl.on_detection_turn(2, -0.60, 1.0, 2.0);
  assert(ctrl.is_armed());

  const auto straight = make_straight_mask();
  const TjunctionMetrics met = analyze_mask(straight.data(), kH, kW, kW, cfg);
  for (int i = 0; i < 6; ++i) {
    const TjunctionDecision dec = ctrl.update(met, true, 1.50 + i * 0.05, true, 2.5 + i * 0.05);
    assert(!dec.active);
    assert(!dec.just_triggered);
  }
  std::cout << "[PASS] test_straight_or_arch_never_triggers\n";
}

void test_perpendicular_wall_triggers_and_exits_closed_loop()
{
  TjunctionConfig cfg;
  cfg.enabled = true;
  cfg.confirm_frames = 2;
  cfg.min_arm_dist_m = 0.30;
  cfg.min_turn_duration_sec = 0.80;
  cfg.max_turn_duration_sec = 1.80;
  TjunctionMergeController ctrl(cfg);
  ctrl.on_detection_turn(2, -0.70, 1.0, 5.0);
  assert(ctrl.is_armed());

  const auto wall = make_perpendicular_wall_mask();
  const TjunctionMetrics wall_m = analyze_mask(wall.data(), kH, kW, kW, cfg);
  assert(!ctrl.update(wall_m, true, 1.35, true, 5.50).active);  // confirm 1/2
  const TjunctionDecision trig = ctrl.update(wall_m, true, 1.40, true, 5.55);
  assert(trig.active && trig.just_triggered);
  assert(trig.angular_z < -1.0);
  assert(trig.suppress_all_red);

  // Mid-turn before min_turn_duration remains active even if straight appears.
  const auto open = make_straight_mask();
  const TjunctionMetrics open_m = analyze_mask(open.data(), kH, kW, kW, cfg);
  assert(ctrl.update(open_m, true, 1.50, true, 6.00).active);

  // Past min_turn_duration with open upper road completes merge cleanly.
  const TjunctionDecision done = ctrl.update(open_m, true, 1.60, true, 6.45);
  assert(!done.active && done.just_completed);
  assert(ctrl.is_done());
  std::cout << "[PASS] test_perpendicular_wall_triggers_and_exits_closed_loop\n";
}

void test_tilted_wall_still_triggers_without_losing_wall()
{
  TjunctionConfig cfg;
  cfg.enabled = true;
  cfg.confirm_frames = 2;
  TjunctionMergeController ctrl(cfg);
  ctrl.on_detection_turn(2, -0.65, 2.0, 10.0);

  // 22-degree tilted camera approach across the boxed T-junction.
  const auto tilted = make_tilted_wall_mask(0.38);
  const TjunctionMetrics m = analyze_mask(tilted.data(), kH, kW, kW, cfg);
  ctrl.update(m, true, 2.40, true, 10.60);
  const TjunctionDecision trig = ctrl.update(m, true, 2.45, true, 10.65);
  assert(trig.active && trig.just_triggered);
  assert(trig.suppress_opposite_red);
  std::cout << "[PASS] test_tilted_wall_still_triggers_without_losing_wall\n";
}

void test_anti_lost_rescue_triggers_immediately_when_line_lost_at_wall()
{
  TjunctionConfig cfg;
  cfg.enabled = true;
  cfg.confirm_frames = 3;
  TjunctionMergeController ctrl(cfg);
  ctrl.on_detection_turn(2, -0.60, 0.5, 1.0);

  std::vector<uint8_t> mostly_dark(kW * kH, 0);
  for (int y = kH - 12; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      mostly_dark[y * kW + x] = 255;
    }
  }
  const TjunctionMetrics m = analyze_mask(mostly_dark.data(), kH, kW, kW, cfg);
  const TjunctionDecision dec = ctrl.update(m, false /* line_detected=false */, 1.0, true, 2.2);
  assert(dec.active && dec.just_triggered && dec.rescue_triggered);
  assert(dec.angular_z < -1.0);
  std::cout << "[PASS] test_anti_lost_rescue_triggers_immediately_when_line_lost_at_wall\n";
}

void test_outer_arch_left_turn_disarms_cleanly()
{
  TjunctionConfig cfg;
  cfg.enabled = true;
  cfg.branch_trigger_sign = -1;
  TjunctionMergeController ctrl(cfg);
  // Left turn (+yaw) onto outer loop -> must disarm, never trigger T-merge on arch.
  ctrl.on_detection_turn(2, +0.65, 1.0, 3.0);
  assert(!ctrl.is_armed());

  const auto wall = make_perpendicular_wall_mask();
  const TjunctionMetrics m = analyze_mask(wall.data(), kH, kW, kW, cfg);
  assert(!ctrl.update(m, true, 1.8, true, 4.5).active);
  std::cout << "[PASS] test_outer_arch_left_turn_disarms_cleanly\n";
}

void test_lap_reset_clears_state()
{
  TjunctionConfig cfg;
  cfg.enabled = true;
  TjunctionMergeController ctrl(cfg);
  ctrl.on_detection_turn(2, -0.65, 1.0, 3.0);
  assert(ctrl.is_armed());
  ctrl.reset_lap();
  assert(!ctrl.is_armed() && !ctrl.is_active() && !ctrl.is_done());
  std::cout << "[PASS] test_lap_reset_clears_state\n";
}

}  // namespace

int main()
{
  test_straight_or_arch_never_triggers();
  test_perpendicular_wall_triggers_and_exits_closed_loop();
  test_tilted_wall_still_triggers_without_losing_wall();
  test_anti_lost_rescue_triggers_immediately_when_line_lost_at_wall();
  test_outer_arch_left_turn_disarms_cleanly();
  test_lap_reset_clears_state();
  std::cout << "ALL 6 T-JUNCTION MERGE TESTS PASSED.\n";
  return 0;
}
