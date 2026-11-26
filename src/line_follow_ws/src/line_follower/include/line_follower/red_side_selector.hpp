#pragma once

#include <algorithm>
#include <cstdint>

namespace line_follower
{

struct RedSideSelection
{
  double side_sign{0.0};
  double left_white_width_px{0.0};
  double right_white_width_px{0.0};
  bool used_white_mask{false};
};

inline RedSideSelection select_red_bypass_side_from_white_mask(
  const uint8_t * white_mask_data,
  int mask_width,
  int mask_height,
  int mask_step,
  int box_x,
  int box_y,
  int box_w,
  int box_h,
  double margin_px,
  int fallback_direction)
{
  const double fallback_sign = (fallback_direction >= 0) ? 1.0 : -1.0;

  if (white_mask_data == nullptr ||
      mask_width <= 0 ||
      mask_height <= 0 ||
      mask_step < mask_width ||
      box_w <= 0 ||
      box_h <= 0)
  {
    return {fallback_sign, 0.0, 0.0, false};
  }

  const int y0 = std::clamp(box_y, 0, mask_height);
  const int y1 = std::clamp(box_y + box_h, 0, mask_height);
  if (y0 >= y1) {
    return {fallback_sign, 0.0, 0.0, false};
  }

  const int x_left_end = std::clamp(box_x, 0, mask_width);
  const int x_right_start = std::clamp(box_x + box_w, 0, mask_width);

  int64_t left_pixels = 0;
  int64_t right_pixels = 0;

  for (int y = y0; y < y1; ++y) {
    const uint8_t * row = white_mask_data + static_cast<size_t>(y) * static_cast<size_t>(mask_step);
    for (int x = 0; x < x_left_end; ++x) {
      if (row[x] > 0) {
        ++left_pixels;
      }
    }
    for (int x = x_right_start; x < mask_width; ++x) {
      if (row[x] > 0) {
        ++right_pixels;
      }
    }
  }

  const double rows = static_cast<double>(y1 - y0);
  const double left_avg = static_cast<double>(left_pixels) / rows;
  const double right_avg = static_cast<double>(right_pixels) / rows;
  const double margin = std::max(0.0, margin_px);

  double side_sign = fallback_sign;
  if (right_avg > left_avg + margin) {
    // 右侧剩余白道更宽 -> 向右绕（现有语义下 side_sign = -1.0）。
    side_sign = -1.0;
  } else if (left_avg > right_avg + margin) {
    // 左侧剩余白道更宽 -> 向左绕（现有语义下 side_sign = 1.0）。
    side_sign = 1.0;
  }

  return {side_sign, left_avg, right_avg, true};
}

}  // namespace line_follower
