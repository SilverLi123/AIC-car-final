#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace line_follower
{

struct RedObstacleBox
{
  int x{0};
  int y{0};
  int w{0};
  int h{0};
  bool is_zone{false};
};

struct RedCarveConfig
{
  int pad_x_far{38};
  int pad_x_near{85};
  double trigger_bottom_ratio{0.22};
  double extend_down_ratio{0.68};
  int min_run_width{20};
  double side_hysteresis_ratio{0.80};
};

struct CarvedRect
{
  int x0{0};
  int y0{0};
  int x1{0};
  int y1{0};
};

struct BandChannelChoice
{
  bool valid{false};
  double center_x{0.0};
  int run_start{0};
  int run_end{0};
  int run_width{0};
};

inline std::vector<CarvedRect> compute_carved_rects(
  const std::vector<RedObstacleBox> & obstacles,
  int roi_w,
  int roi_h,
  const RedCarveConfig & cfg,
  double pad_scale = 1.0)
{
  std::vector<CarvedRect> rects;
  if (roi_w <= 0 || roi_h <= 0) {
    return rects;
  }

  const double scale = std::max(0.0, pad_scale);
  const double roi_h_d = static_cast<double>(roi_h);

  rects.reserve(obstacles.size());
  for (const auto & obs : obstacles) {
    if (obs.w <= 0 || obs.h <= 0) {
      continue;
    }

    const double y_rel = std::max(
      0.0,
      std::min(1.0, static_cast<double>(obs.y + obs.h) / roi_h_d));

    const int pad_x = std::max(
      0,
      static_cast<int>(std::llround(
        (static_cast<double>(cfg.pad_x_far) +
         y_rel * static_cast<double>(cfg.pad_x_near - cfg.pad_x_far)) * scale)));

    const int x0 = std::clamp(obs.x - pad_x, 0, roi_w);
    const int x1 = std::clamp(obs.x + obs.w + pad_x, 0, roi_w);
    const int y0 = std::clamp(obs.y - 6, 0, roi_h);

    int y1_raw = obs.y + obs.h;
    if (y_rel >= cfg.trigger_bottom_ratio) {
      const int ext_px = std::max(
        0,
        static_cast<int>(std::llround(cfg.extend_down_ratio * roi_h_d)));
      y1_raw = std::min(roi_h, obs.y + obs.h + ext_px);
    }
    const int y1 = std::clamp(y1_raw, 0, roi_h);

    if (x1 > x0 && y1 > y0) {
      rects.push_back({x0, y0, x1, y1});
    }
  }

  return rects;
}

inline BandChannelChoice select_widest_band_channel(
  const uint8_t * mask_data,
  int width,
  int height,
  int step,
  int y0,
  int y1,
  int min_run_width,
  bool has_prev_x,
  double prev_x,
  double hysteresis_ratio)
{
  if (mask_data == nullptr || width <= 0 || height <= 0 || step < width) {
    return {};
  }

  const int cy0 = std::clamp(y0, 0, height - 1);
  const int cy1 = std::clamp(y1, cy0 + 1, height);
  const int band_h = cy1 - cy0;
  if (band_h <= 0) {
    return {};
  }

  const int min_white_pixels = std::max(
    1,
    static_cast<int>(static_cast<double>(band_h) * 0.30));

  std::vector<uint8_t> active(static_cast<size_t>(width), 0);
  for (int x = 0; x < width; ++x) {
    int white_cnt = 0;
    for (int y = cy0; y < cy1; ++y) {
      const uint8_t * row = mask_data + static_cast<size_t>(y) * static_cast<size_t>(step);
      if (row[x] > 0) {
        ++white_cnt;
      }
    }
    if (white_cnt >= min_white_pixels) {
      active[static_cast<size_t>(x)] = 1;
    }
  }

  auto collect_runs = [&](int req_width) -> std::vector<BandChannelChoice> {
    std::vector<BandChannelChoice> runs;
    int run_start = -1;
    for (int x = 0; x <= width; ++x) {
      const bool is_active =
        (x < width) ? (active[static_cast<size_t>(x)] != 0) : false;
      if (is_active && run_start < 0) {
        run_start = x;
      }
      if ((!is_active || x == width) && run_start >= 0) {
        const int run_end = x - 1;
        const int run_w = run_end - run_start + 1;
        if (run_w >= req_width) {
          const double center = 0.5 * static_cast<double>(run_start + run_end);
          runs.push_back({true, center, run_start, run_end, run_w});
        }
        run_start = -1;
      }
    }
    return runs;
  };

  auto runs = collect_runs(std::max(1, min_run_width));
  if (runs.empty()) {
    runs = collect_runs(std::max(8, min_run_width / 2));
  }
  if (runs.empty()) {
    return {};
  }

  const double mid_x = 0.5 * static_cast<double>(width);
  size_t widest_idx = 0;
  for (size_t i = 1; i < runs.size(); ++i) {
    if (runs[i].run_width > runs[widest_idx].run_width) {
      widest_idx = i;
    } else if (runs[i].run_width == runs[widest_idx].run_width &&
               std::abs(runs[i].center_x - mid_x) < std::abs(runs[widest_idx].center_x - mid_x))
    {
      widest_idx = i;
    }
  }
  const BandChannelChoice & widest = runs[widest_idx];

  if (has_prev_x && std::isfinite(prev_x)) {
    size_t closest_idx = 0;
    double best_dist = std::abs(runs[0].center_x - prev_x);
    for (size_t i = 1; i < runs.size(); ++i) {
      const double d = std::abs(runs[i].center_x - prev_x);
      if (d < best_dist) {
        best_dist = d;
        closest_idx = i;
      }
    }
    const BandChannelChoice & closest = runs[closest_idx];
    const double hyst = std::max(0.5, std::min(hysteresis_ratio, 1.0));
    if (static_cast<double>(closest.run_width) >= static_cast<double>(widest.run_width) * hyst &&
        best_dist <= 0.35 * static_cast<double>(width))
    {
      return closest;
    }
  }

  return widest;
}

}  // namespace line_follower
