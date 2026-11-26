#include <cassert>
#include <cmath>
#include <iostream>

#include "line_follower/steering_pd_controller.hpp"

using line_follower::SteeringPdConfig;
using line_follower::SteeringPdController;

namespace
{

double clamp_ref(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

void test_kd_zero_matches_pure_p()
{
  SteeringPdController pd;
  SteeringPdConfig cfg;
  cfg.kp = 2.45;
  cfg.kd = 0.0;
  cfg.max_yaw_rate = 1.5;

  const double errors[] = {0.10, 0.35, -0.20, 0.80, -0.90};
  double t = 1.0;
  for (double err : errors) {
    const double yaw = pd.compute(err, t, cfg);
    const double pure_p = clamp_ref(-(cfg.kp * err), -cfg.max_yaw_rate, cfg.max_yaw_rate);
    assert(std::abs(yaw - pure_p) < 1e-12);
    t += 0.03;
  }
  std::cout << "[PASS] test_kd_zero_matches_pure_p\n";
}

void test_decreasing_error_adds_damping_against_overshoot()
{
  SteeringPdController pd;
  SteeringPdConfig cfg;
  cfg.kp = 2.0;
  cfg.kd = 0.20;
  cfg.kd_lpf_alpha = 1.0;  // pure finite difference for deterministic check
  cfg.kd_max_contribution = 2.0;
  cfg.max_yaw_rate = 3.0;

  // Frame 1: initialized, zero derivative kick.
  const double y0 = pd.compute(0.40, 1.00, cfg);
  assert(std::abs(y0 - (-0.80)) < 1e-9);

  // Frame 2: error shrinks +0.40 -> +0.20 in 0.05s => raw_d = -4.0.
  // P = -0.40, D = -0.20 * (-4.0) = +0.80 => total = +0.40 (counter-steers to prevent overshoot).
  const double y1 = pd.compute(0.20, 1.05, cfg);
  assert(y1 > -0.40);
  assert(std::abs(y1 - 0.40) < 1e-9);
  std::cout << "[PASS] test_decreasing_error_adds_damping_against_overshoot\n";
}

void test_derivative_contribution_clamped_on_step_jump()
{
  SteeringPdController pd;
  SteeringPdConfig cfg;
  cfg.kp = 2.0;
  cfg.kd = 0.50;
  cfg.kd_lpf_alpha = 1.0;
  cfg.kd_max_contribution = 0.45;
  cfg.max_yaw_rate = 2.5;

  pd.compute(0.0, 1.00, cfg);
  // Sudden +1.0 error jump in 0.02s => raw_d = +50.0, unclamped D = -25.0, clamped to -0.45.
  // P = -2.0 * 1.0 = -2.0; total = -2.45.
  const double yaw = pd.compute(1.0, 1.02, cfg);
  assert(std::abs(yaw - (-2.45)) < 1e-9);
  std::cout << "[PASS] test_derivative_contribution_clamped_on_step_jump\n";
}

void test_reset_and_dt_timeout_prevent_derivative_kick()
{
  SteeringPdController pd;
  SteeringPdConfig cfg;
  cfg.kp = 2.0;
  cfg.kd = 0.25;
  cfg.kd_lpf_alpha = 0.5;
  cfg.kd_max_contribution = 1.0;
  cfg.max_yaw_rate = 2.0;
  cfg.max_dt_sec = 0.25;

  pd.compute(0.10, 1.00, cfg);
  pd.compute(0.20, 1.03, cfg);
  assert(std::abs(pd.last_d_filtered()) > 0.0);

  // Explicit reset clears state so next frame acts as pure P.
  pd.reset();
  assert(std::abs(pd.last_d_filtered()) < 1e-12);
  const double after_reset = pd.compute(0.50, 1.06, cfg);
  assert(std::abs(after_reset - (-1.00)) < 1e-9);

  // Build up derivative, then simulate dt > max_dt_sec (gap/frame drop).
  pd.compute(0.60, 1.09, cfg);
  assert(std::abs(pd.last_d_filtered()) > 0.0);
  const double after_gap = pd.compute(-0.20, 1.50, cfg);
  assert(std::abs(pd.last_d_filtered()) < 1e-12);
  assert(std::abs(after_gap - 0.40) < 1e-9);
  std::cout << "[PASS] test_reset_and_dt_timeout_prevent_derivative_kick\n";
}

void test_lpf_attenuates_high_frequency_jitter()
{
  SteeringPdController pd_raw;
  SteeringPdController pd_lpf;
  SteeringPdConfig cfg_raw;
  cfg_raw.kp = 2.0;
  cfg_raw.kd = 0.15;
  cfg_raw.kd_lpf_alpha = 1.0;
  cfg_raw.kd_max_contribution = 5.0;
  cfg_raw.max_yaw_rate = 5.0;

  SteeringPdConfig cfg_lpf = cfg_raw;
  cfg_lpf.kd_lpf_alpha = 0.35;

  const double dt = 0.03;
  double t = 0.0;
  pd_raw.compute(0.0, t, cfg_raw);
  pd_lpf.compute(0.0, t, cfg_lpf);

  // First spike: +0.10 -> raw_d = 0.10 / 0.03 = 3.333.
  // LPF scales the step response amplitude cleanly below raw finite difference.
  t += dt;
  pd_raw.compute(0.10, t, cfg_raw);
  pd_lpf.compute(0.10, t, cfg_lpf);
  assert(std::abs(pd_lpf.last_d_filtered()) < std::abs(pd_raw.last_d_filtered()));

  // Alternating jitter back to 0.0: raw derivative flips sharply negative,
  // while filtered derivative stays attenuated.
  t += dt;
  pd_raw.compute(0.0, t, cfg_raw);
  pd_lpf.compute(0.0, t, cfg_lpf);
  assert(std::abs(pd_lpf.last_d_filtered()) < std::abs(pd_raw.last_d_filtered()));
  std::cout << "[PASS] test_lpf_attenuates_high_frequency_jitter\n";
}

}  // namespace

int main()
{
  test_kd_zero_matches_pure_p();
  test_decreasing_error_adds_damping_against_overshoot();
  test_derivative_contribution_clamped_on_step_jump();
  test_reset_and_dt_timeout_prevent_derivative_kick();
  test_lpf_attenuates_high_frequency_jitter();
  std::cout << "All 5 steering_pd_controller unit tests passed.\n";
  return 0;
}
