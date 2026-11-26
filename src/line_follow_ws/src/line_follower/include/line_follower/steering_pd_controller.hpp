#pragma once

#include <algorithm>
#include <cmath>

namespace line_follower
{

struct SteeringPdConfig
{
  double kp{1.8};
  double kd{0.0};
  double kd_lpf_alpha{0.35};
  double kd_max_contribution{0.45};
  double max_yaw_rate{1.2};
  double min_dt_sec{1e-4};
  double max_dt_sec{0.25};
};

class SteeringPdController
{
public:
  void reset()
  {
    initialized_ = false;
    prev_error_ = 0.0;
    prev_time_sec_ = 0.0;
    d_filtered_ = 0.0;
  }

  double compute(double error, double now_sec, const SteeringPdConfig & cfg)
  {
    const double max_yaw = std::max(0.0, cfg.max_yaw_rate);
    const double p_term = -(cfg.kp * error);

    if (cfg.kd <= 0.0 || !std::isfinite(error) || !std::isfinite(now_sec)) {
      reset();
      return clamp_value(p_term, -max_yaw, max_yaw);
    }

    double d_term = 0.0;
    if (!initialized_) {
      initialized_ = true;
      prev_error_ = error;
      prev_time_sec_ = now_sec;
      d_filtered_ = 0.0;
    } else {
      const double dt = now_sec - prev_time_sec_;
      if (dt < cfg.min_dt_sec || dt > cfg.max_dt_sec) {
        prev_error_ = error;
        prev_time_sec_ = now_sec;
        d_filtered_ = 0.0;
      } else {
        const double raw_d = (error - prev_error_) / dt;
        const double alpha = clamp_value(cfg.kd_lpf_alpha, 0.05, 1.0);
        d_filtered_ = alpha * raw_d + (1.0 - alpha) * d_filtered_;
        const double max_d = std::max(0.0, cfg.kd_max_contribution);
        d_term = clamp_value(-(cfg.kd * d_filtered_), -max_d, max_d);
        prev_error_ = error;
        prev_time_sec_ = now_sec;
      }
    }

    return clamp_value(p_term + d_term, -max_yaw, max_yaw);
  }

  double last_d_filtered() const
  {
    return d_filtered_;
  }

private:
  static double clamp_value(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  bool initialized_{false};
  double prev_error_{0.0};
  double prev_time_sec_{0.0};
  double d_filtered_{0.0};
};

}  // namespace line_follower
