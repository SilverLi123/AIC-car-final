#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace line_follower
{
namespace tjunction
{

struct TjunctionConfig
{
  bool enabled{false};
  bool require_branch_gate{true};
  int arm_yellow_count{2};
  int branch_trigger_sign{-1};   // <0: arms on right turn into shortcut crossbar; >0: left
  int merge_direction{-1};       // -1: right turn downward at T-junction wall; +1: left
  double min_arm_dist_m{0.32};
  double min_arm_time_sec{1.0};
  int confirm_frames{2};
  double max_upper_road_ratio{0.26};
  double min_wide_row_ratio{0.28};
  double min_row_span_ratio{0.72};
  double left_edge_max_ratio{0.22};
  double right_edge_min_ratio{0.78};
  double min_top_dark_ratio{0.70};
  double rescue_upper_road_ratio{0.12};
  double rescue_total_road_ratio{0.28};
  double merge_speed{0.14};
  double merge_yaw_rate{1.45};
  double min_turn_duration_sec{0.85};
  double max_turn_duration_sec{1.85};
  double exit_upper_road_ratio{0.30};
  double exit_max_wide_row_ratio{0.35};
};

struct TjunctionMetrics
{
  double upper_center_road_ratio{1.0};
  double lower_wide_row_ratio{0.0};
  double top_wall_dark_ratio{0.0};
  double total_road_ratio{1.0};
  int wide_rows{0};
};

struct TjunctionDecision
{
  bool active{false};
  bool suppress_opposite_red{false};
  bool suppress_all_red{false};
  bool just_triggered{false};
  bool just_completed{false};
  bool rescue_triggered{false};
  double linear_x{0.0};
  double angular_z{0.0};
};

/**
 * Compute tilt- and occlusion-tolerant T-junction wall geometry from a
 * bottom-connected road(+bridged-red) mask.
 */
inline TjunctionMetrics analyze_mask(
  const uint8_t * data,
  int rows,
  int cols,
  int stride,
  const TjunctionConfig & cfg)
{
  TjunctionMetrics m;
  if (data == nullptr || rows <= 0 || cols <= 0 || stride < cols) {
    return m;
  }

  // 1. Total road ratio across the full ROI.
  int total_white = 0;
  for (int y = 0; y < rows; ++y) {
    const uint8_t * row = data + y * stride;
    for (int x = 0; x < cols; ++x) {
      if (row[x] > 0) {
        total_white++;
      }
    }
  }
  m.total_road_ratio = static_cast<double>(total_white) / static_cast<double>(rows * cols);

  // 2. Upper-center window [0.05..0.40 H, 0.20..0.80 W]: checks if straight road exists ahead.
  const int uc_y0 = std::clamp(static_cast<int>(rows * 0.05), 0, rows - 1);
  const int uc_y1 = std::clamp(static_cast<int>(rows * 0.40), uc_y0 + 1, rows);
  const int uc_x0 = std::clamp(static_cast<int>(cols * 0.20), 0, cols - 1);
  const int uc_x1 = std::clamp(static_cast<int>(cols * 0.80), uc_x0 + 1, cols);
  int uc_white = 0;
  for (int y = uc_y0; y < uc_y1; ++y) {
    const uint8_t * row = data + y * stride;
    for (int x = uc_x0; x < uc_x1; ++x) {
      if (row[x] > 0) {
        uc_white++;
      }
    }
  }
  const int uc_total = (uc_y1 - uc_y0) * (uc_x1 - uc_x0);
  m.upper_center_road_ratio =
    uc_total > 0 ? static_cast<double>(uc_white) / static_cast<double>(uc_total) : 1.0;

  // 3. Lower wide rows [0.35..1.00 H]: tilted-wall tolerant span detector.
  const int lower_y0 = std::clamp(static_cast<int>(rows * 0.35), 0, rows - 1);
  const int min_span = std::max(1, static_cast<int>(std::round(cols * cfg.min_row_span_ratio)));
  const int max_left_x = static_cast<int>(std::round(cols * cfg.left_edge_max_ratio));
  const int min_right_x = static_cast<int>(std::round(cols * cfg.right_edge_min_ratio));

  int first_wide_row = -1;
  int wide_count = 0;
  for (int y = lower_y0; y < rows; ++y) {
    const uint8_t * row = data + y * stride;
    int left_x = -1;
    int right_x = -1;
    for (int x = 0; x < cols; ++x) {
      if (row[x] > 0) {
        if (left_x < 0) {
          left_x = x;
        }
        right_x = x;
      }
    }
    if (left_x >= 0 && left_x <= max_left_x && right_x >= min_right_x &&
        (right_x - left_x + 1) >= min_span)
    {
      if (first_wide_row < 0) {
        first_wide_row = y;
      }
      wide_count++;
    }
  }
  m.wide_rows = wide_count;
  m.lower_wide_row_ratio =
    static_cast<double>(wide_count) / static_cast<double>(std::max(1, rows - lower_y0));

  // 4. Darkness immediately above the (possibly tilted) T-wall top edge, per column.
  // Tracing upward from bottom row handles angled approach across the boxed crossbar.
  if (first_wide_row > 4) {
    const int band_h = std::max(8, static_cast<int>(rows * 0.20));
    int band_dark = 0;
    int band_total = 0;
    for (int x = 0; x < cols; ++x) {
      int top_y = rows;
      int gap = 0;
      bool started = false;
      for (int y = rows - 1; y >= 0; --y) {
        if (data[y * stride + x] > 0) {
          top_y = y;
          started = true;
          gap = 0;
        } else if (started) {
          gap++;
          if (gap > 3) {
            break;
          }
        }
      }
      if (!started || top_y <= 2) {
        continue;
      }
      const int y0 = std::max(0, top_y - band_h);
      for (int y = y0; y < top_y; ++y) {
        band_total++;
        if (data[y * stride + x] == 0) {
          band_dark++;
        }
      }
    }
    m.top_wall_dark_ratio =
      band_total > 0 ? (static_cast<double>(band_dark) / static_cast<double>(band_total)) : (1.0 - m.upper_center_road_ratio);
  } else {
    m.top_wall_dark_ratio = 1.0 - m.upper_center_road_ratio;
  }

  return m;
}

class TjunctionMergeController
{
public:
  explicit TjunctionMergeController(const TjunctionConfig & cfg = TjunctionConfig{})
  : cfg_(cfg)
  {
  }

  void set_config(const TjunctionConfig & cfg)
  {
    cfg_ = cfg;
  }

  const TjunctionConfig & config() const
  {
    return cfg_;
  }

  void reset_lap()
  {
    branch_decided_ = false;
    armed_ = false;
    done_ = false;
    active_ = false;
    confirm_count_ = 0;
    arm_dist_m_ = 0.0;
    arm_time_sec_ = 0.0;
    start_time_sec_ = 0.0;
  }

  void on_pause()
  {
    confirm_count_ = 0;
    active_ = false;
  }

  bool is_armed() const { return armed_; }
  bool is_active() const { return active_; }
  bool is_done() const { return done_; }

  void on_detection_turn(
    int yellow_count,
    double angular_z,
    double odom_dist_m,
    double now_sec)
  {
    if (!cfg_.enabled || branch_decided_ || yellow_count != cfg_.arm_yellow_count) {
      return;
    }
    if (std::abs(angular_z) < 0.05) {
      return;
    }

    branch_decided_ = true;
    const bool matches =
      (cfg_.branch_trigger_sign < 0 && angular_z < -0.05) ||
      (cfg_.branch_trigger_sign > 0 && angular_z > 0.05);
    if (matches) {
      armed_ = true;
      arm_dist_m_ = odom_dist_m;
      arm_time_sec_ = now_sec;
      confirm_count_ = 0;
    } else {
      armed_ = false;
    }
  }

  TjunctionDecision update(
    const TjunctionMetrics & m,
    bool line_detected,
    double odom_dist_m,
    bool odom_valid,
    double now_sec)
  {
    TjunctionDecision dec;
    if (!cfg_.enabled || done_) {
      return dec;
    }

    const double dir_sign = (cfg_.merge_direction < 0) ? -1.0 : 1.0;
    const double merge_yaw = dir_sign * std::abs(cfg_.merge_yaw_rate);

    if (active_) {
      const double elapsed = std::max(0.0, now_sec - start_time_sec_);
      const bool road_open =
        line_detected &&
        elapsed >= cfg_.min_turn_duration_sec &&
        m.upper_center_road_ratio >= cfg_.exit_upper_road_ratio &&
        m.lower_wide_row_ratio <= cfg_.exit_max_wide_row_ratio;
      if (road_open || elapsed >= cfg_.max_turn_duration_sec) {
        active_ = false;
        armed_ = false;
        done_ = true;
        confirm_count_ = 0;
        dec.just_completed = true;
        return dec;
      }

      dec.active = true;
      dec.suppress_opposite_red = true;
      dec.suppress_all_red = (elapsed < cfg_.min_turn_duration_sec);
      dec.linear_x = cfg_.merge_speed;
      dec.angular_z = merge_yaw;
      return dec;
    }

    const bool gate_ok = !cfg_.require_branch_gate || armed_;
    if (!gate_ok) {
      return dec;
    }

    const bool travel_ready = !cfg_.require_branch_gate ||
      (odom_valid ? ((odom_dist_m - arm_dist_m_) >= cfg_.min_arm_dist_m)
                  : ((now_sec - arm_time_sec_) >= cfg_.min_arm_time_sec));
    if (!travel_ready) {
      return dec;
    }

    if (m.upper_center_road_ratio <= cfg_.max_upper_road_ratio + 0.06) {
      dec.suppress_opposite_red = true;
    }

    const bool standard_wall =
      m.upper_center_road_ratio <= cfg_.max_upper_road_ratio &&
      m.lower_wide_row_ratio >= cfg_.min_wide_row_ratio &&
      m.top_wall_dark_ratio >= cfg_.min_top_dark_ratio;

    const bool rescue_wall =
      !line_detected ||
      (m.upper_center_road_ratio <= cfg_.rescue_upper_road_ratio &&
       m.total_road_ratio <= cfg_.rescue_total_road_ratio);

    if (rescue_wall) {
      active_ = true;
      start_time_sec_ = now_sec;
      confirm_count_ = 0;
      dec.active = true;
      dec.just_triggered = true;
      dec.rescue_triggered = true;
      dec.suppress_opposite_red = true;
      dec.suppress_all_red = true;
      dec.linear_x = cfg_.merge_speed;
      dec.angular_z = merge_yaw;
      return dec;
    }

    if (standard_wall) {
      confirm_count_++;
      dec.suppress_opposite_red = true;
      if (confirm_count_ >= std::max(1, cfg_.confirm_frames)) {
        active_ = true;
        start_time_sec_ = now_sec;
        confirm_count_ = 0;
        dec.active = true;
        dec.just_triggered = true;
        dec.suppress_all_red = true;
        dec.linear_x = cfg_.merge_speed;
        dec.angular_z = merge_yaw;
        return dec;
      }
    } else {
      confirm_count_ = std::max(0, confirm_count_ - 1);
    }

    return dec;
  }

private:
  TjunctionConfig cfg_;
  bool branch_decided_{false};
  bool armed_{false};
  bool done_{false};
  bool active_{false};
  int confirm_count_{0};
  double arm_dist_m_{0.0};
  double arm_time_sec_{0.0};
  double start_time_sec_{0.0};
};

}  // namespace tjunction
}  // namespace line_follower
