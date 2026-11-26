#pragma once

#include <algorithm>

namespace line_follower
{

struct RedLatAccelConfig
{
  double max_yaw_rate{1.55};
  double red_avoid_max_yaw_rate{9.9};
  double max_lat_accel{0.0};
  double speed_floor{0.18};
  double highspeed_slew_ref_speed{0.0};
  double min_slew_scale{0.45};
};

/**
 * Use the larger of commanded speed and fresh odometry speed so braking lag at
 * high-speed obstacle entry still enforces the tighter lateral-accel cap.
 */
inline double compute_effective_speed(double cmd_speed, double odom_speed, bool odom_fresh)
{
  const double v_cmd = std::max(0.0, cmd_speed);
  const double v_odom = odom_fresh ? std::max(0.0, odom_speed) : 0.0;
  return std::max(v_cmd, v_odom);
}

/**
 * Cap red-avoidance angular velocity by lateral acceleration a_y = v * w while
 * preserving slow-speed tight-bend authority up to red_avoid_max_yaw_rate.
 */
inline double compute_red_avoid_yaw_cap(const RedLatAccelConfig & cfg, double v_eff)
{
  const double base_cap = std::min(
    std::max(0.2, cfg.max_yaw_rate),
    std::max(0.2, cfg.red_avoid_max_yaw_rate));
  if (cfg.max_lat_accel <= 0.0) {
    return base_cap;
  }
  const double v_denom = std::max(std::max(0.0, v_eff), std::max(0.05, cfg.speed_floor));
  const double lat_cap = std::max(0.25, cfg.max_lat_accel / v_denom);
  return std::min(base_cap, lat_cap);
}

/**
 * Scale red-avoidance yaw slew rate inversely with effective speed above the
 * reference speed so high-speed obstacle entry steers smoothly instead of whipping.
 */
inline double compute_red_avoid_slew_rate(
  double base_slew_rate, const RedLatAccelConfig & cfg, double v_eff)
{
  if (base_slew_rate <= 0.0) {
    return 0.0;
  }
  if (cfg.highspeed_slew_ref_speed <= 0.0) {
    return base_slew_rate;
  }
  const double ref = cfg.highspeed_slew_ref_speed;
  const double scale = std::clamp(
    ref / std::max(std::max(0.0, v_eff), ref),
    std::clamp(cfg.min_slew_scale, 0.1, 1.0),
    1.0);
  return base_slew_rate * scale;
}

}  // namespace line_follower
