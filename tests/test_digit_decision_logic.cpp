#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "line_follower/digit_decision_logic.hpp"

using line_follower::decision::DigitCandidate;
using line_follower::decision::RobustDecisionConfig;
using line_follower::decision::RobustDigitDecision;
using line_follower::decision::TurnPlan;

namespace
{

void test_pair_selection_picks_larger_digit()
{
  RobustDigitDecision dec;
  dec.on_yellow_line(10.0);

  // Frame 1: left=3 (x=0.35), right=6 (x=0.68)
  TurnPlan p1 = dec.process_frame(
    {{3, 0.85, 0.35, 0.45}, {6, 0.80, 0.68, 0.45}}, 0.50, 10.5);
  assert(!p1.trigger);

  // Frame 2: votes reach 2 -> should choose 6 on the right (x=0.68 > 0.50 -> negative yaw with sign -1)
  TurnPlan p2 = dec.process_frame(
    {{3, 0.86, 0.36, 0.50}, {6, 0.82, 0.67, 0.50}}, 0.50, 10.6);
  assert(p2.trigger);
  assert(p2.chosen_digit == 6);
  assert(p2.is_fork);
  assert(p2.angular_z < 0.0);
  std::cout << "[PASS] test_pair_selection_picks_larger_digit\n";
}

void test_single_frame_low_conf_8_rejected()
{
  RobustDigitDecision dec;
  dec.on_yellow_line(10.0);

  // Frame 1: real digit 3 (high conf) + bogus low-conf 8 (below 0.45 and single frame)
  TurnPlan p1 = dec.process_frame(
    {{3, 0.88, 0.34, 0.45}, {8, 0.22, 0.70, 0.45}}, 0.50, 10.5);
  assert(!p1.trigger);

  // Frame 2: real digit 3 again + high-conf 8 for only 1 frame (only 1 vote < min_votes=2)
  TurnPlan p2 = dec.process_frame(
    {{3, 0.90, 0.35, 0.52}, {8, 0.55, 0.70, 0.52}}, 0.50, 10.6);
  assert(p2.trigger);
  assert(p2.chosen_digit == 3);
  assert(p2.angular_z > 0.0);
  std::cout << "[PASS] test_single_frame_low_conf_8_rejected\n";
}

void test_high_speed_y_skip_still_triggers()
{
  RobustDigitDecision dec;
  // Straight section (no recent yellow line)
  // Frame 1 at y=0.34, Frame 2 jumps straight to y=0.70 (would fail old y_align_tolerance=0.01)
  TurnPlan p1 = dec.process_frame({{5, 0.80, 0.38, 0.34}}, 0.50, 20.0);
  assert(!p1.trigger);
  TurnPlan p2 = dec.process_frame({{5, 0.82, 0.37, 0.70}}, 0.50, 20.05);
  assert(p2.trigger);
  assert(p2.chosen_digit == 5);
  assert(!p2.is_fork);
  std::cout << "[PASS] test_high_speed_y_skip_still_triggers\n";
}

void test_fork_vs_straight_limits()
{
  RobustDigitDecision dec;
  RobustDecisionConfig cfg;
  cfg.fork_max_yaw_rate = 1.3;
  cfg.fork_turn_duration = 0.38;
  cfg.straight_max_yaw_rate = 0.55;
  cfg.straight_turn_duration = 0.25;
  cfg.yaw_gain = 4.0;  // large gain to hit saturation limits
  dec.set_config(cfg);

  // Post-yellow -> fork limits
  dec.on_yellow_line(10.0);
  dec.process_frame({{7, 0.90, 0.15, 0.50}}, 0.50, 11.0);
  TurnPlan p_fork = dec.process_frame({{7, 0.90, 0.15, 0.55}}, 0.50, 11.1);
  assert(p_fork.trigger && p_fork.is_fork);
  assert(std::abs(p_fork.angular_z - 1.3) < 1e-6);
  assert(std::abs(p_fork.duration_sec - 0.38) < 1e-6);

  // Far after yellow (>3.5s) -> straight limits
  dec.process_frame({{4, 0.90, 0.15, 0.50}}, 0.50, 16.0);
  TurnPlan p_str = dec.process_frame({{4, 0.90, 0.15, 0.55}}, 0.50, 16.1);
  assert(p_str.trigger && !p_str.is_fork);
  assert(std::abs(p_str.angular_z - 0.55) < 1e-6);
  assert(std::abs(p_str.duration_sec - 0.25) < 1e-6);
  std::cout << "[PASS] test_fork_vs_straight_limits\n";
}

void test_closed_loop_early_finish()
{
  RobustDigitDecision dec;
  dec.on_yellow_line(1.0);
  dec.process_frame({{6, 0.85, 0.72, 0.50}}, 0.50, 1.5);
  TurnPlan p = dec.process_frame({{6, 0.85, 0.70, 0.55}}, 0.50, 1.6);
  assert(p.trigger);

  // Not yet centered
  assert(!dec.should_finish_turn_early({{6, 0.85, 0.63, 0.65}}));
  // Aligned near center & bottom -> finish immediately
  assert(dec.should_finish_turn_early({{6, 0.85, 0.52, 0.75}}));

  // Also verify disappearance after passing bottom finishes early
  dec.process_frame({{6, 0.85, 0.72, 0.60}}, 0.50, 5.0);
  TurnPlan p2 = dec.process_frame({{6, 0.85, 0.70, 0.74}}, 0.50, 5.1);
  assert(p2.trigger);
  assert(!dec.should_finish_turn_early({}));  // missing frame 1
  assert(dec.should_finish_turn_early({}));   // missing frame 2 -> finish
  std::cout << "[PASS] test_closed_loop_early_finish\n";
}

void test_no_digits_never_triggers()
{
  RobustDigitDecision dec;
  for (int i = 0; i < 6; ++i) {
    TurnPlan p = dec.process_frame({}, 0.50, 1.0 + i * 0.05);
    assert(!p.trigger);
  }
  // Out-of-band detections (top of image y=0.10) do not trigger
  TurnPlan p_top1 = dec.process_frame({{8, 0.99, 0.20, 0.10}}, 0.50, 2.0);
  TurnPlan p_top2 = dec.process_frame({{8, 0.99, 0.20, 0.15}}, 0.50, 2.1);
  assert(!p_top1.trigger && !p_top2.trigger);
  std::cout << "[PASS] test_no_digits_never_triggers\n";
}

}  // namespace

int main()
{
  test_pair_selection_picks_larger_digit();
  test_single_frame_low_conf_8_rejected();
  test_high_speed_y_skip_still_triggers();
  test_fork_vs_straight_limits();
  test_closed_loop_early_finish();
  test_no_digits_never_triggers();
  std::cout << "ALL 6 DIGIT DECISION TESTS PASSED\n";
  return 0;
}
