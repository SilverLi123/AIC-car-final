#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "line_follower/red_side_selector.hpp"

using line_follower::RedSideSelection;
using line_follower::select_red_bypass_side_from_white_mask;

namespace
{

std::vector<uint8_t> make_mask_with_white_span(
  int height,
  int step,
  int y0,
  int y1,
  int white_x0,
  int white_x1)
{
  std::vector<uint8_t> buf(static_cast<size_t>(step * height), 0);
  for (int y = y0; y < y1; ++y) {
    for (int x = white_x0; x < white_x1; ++x) {
      buf[static_cast<size_t>(y * step + x)] = 255;
    }
  }
  return buf;
}

void test_picks_wider_right_white_lane_even_when_left_img_margin_larger()
{
  // 100x20 mask with step=128 (non-contiguous stride test).
  // Red box spans x=[55, 70), y=[4, 14):
  //   left image margin = 55 px, right image margin = 30 px (old logic would pick left).
  // White road covers x=[45, 98):
  //   left of box white width = [45, 55) = 10 px
  //   right of box white width = [70, 98) = 28 px -> right is wider, must steer right (-1.0).
  const int w = 100;
  const int h = 20;
  const int step = 128;
  const auto mask = make_mask_with_white_span(h, step, 0, h, 45, 98);

  const RedSideSelection sel = select_red_bypass_side_from_white_mask(
    mask.data(), w, h, step, 55, 4, 15, 10, 5.0, 1);

  assert(sel.used_white_mask);
  assert(std::abs(sel.left_white_width_px - 10.0) < 1e-6);
  assert(std::abs(sel.right_white_width_px - 28.0) < 1e-6);
  assert(sel.side_sign == -1.0);
  std::cout << "[PASS] test_picks_wider_right_white_lane_even_when_left_img_margin_larger\n";
}

void test_picks_wider_left_white_lane_even_when_right_img_margin_larger()
{
  // Red box spans x=[30, 45), y=[2, 12):
  //   left image margin = 30 px, right image margin = 55 px (old logic would pick right).
  // White road covers x=[2, 55):
  //   left of box white width = [2, 30) = 28 px
  //   right of box white width = [45, 55) = 10 px -> left is wider, must steer left (+1.0).
  const int w = 100;
  const int h = 20;
  const int step = 100;
  const auto mask = make_mask_with_white_span(h, step, 0, h, 2, 55);

  const RedSideSelection sel = select_red_bypass_side_from_white_mask(
    mask.data(), w, h, step, 30, 2, 15, 10, 5.0, -1);

  assert(sel.used_white_mask);
  assert(std::abs(sel.left_white_width_px - 28.0) < 1e-6);
  assert(std::abs(sel.right_white_width_px - 10.0) < 1e-6);
  assert(sel.side_sign == 1.0);
  std::cout << "[PASS] test_picks_wider_left_white_lane_even_when_right_img_margin_larger\n";
}

void test_within_margin_uses_fallback_direction()
{
  // Red box spans x=[40, 60), y=[5, 15), white span covers x=[18, 86):
  //   left white = 22 px, right white = 26 px, diff = 4 px <= margin (10.0).
  const int w = 100;
  const int h = 20;
  const auto mask = make_mask_with_white_span(h, w, 0, h, 18, 86);

  const RedSideSelection sel_neg = select_red_bypass_side_from_white_mask(
    mask.data(), w, h, w, 40, 5, 20, 10, 10.0, -1);
  assert(sel_neg.used_white_mask);
  assert(sel_neg.side_sign == -1.0);

  const RedSideSelection sel_pos = select_red_bypass_side_from_white_mask(
    mask.data(), w, h, w, 40, 5, 20, 10, 10.0, 1);
  assert(sel_pos.used_white_mask);
  assert(sel_pos.side_sign == 1.0);
  std::cout << "[PASS] test_within_margin_uses_fallback_direction\n";
}

void test_invalid_inputs_fall_back_safely()
{
  const std::vector<uint8_t> dummy(100, 255);

  const RedSideSelection s_null = select_red_bypass_side_from_white_mask(
    nullptr, 10, 10, 10, 2, 2, 4, 4, 10.0, -1);
  assert(!s_null.used_white_mask);
  assert(s_null.side_sign == -1.0);

  const RedSideSelection s_empty = select_red_bypass_side_from_white_mask(
    dummy.data(), 0, 10, 10, 2, 2, 4, 4, 10.0, 1);
  assert(!s_empty.used_white_mask);
  assert(s_empty.side_sign == 1.0);

  const RedSideSelection s_zero_box = select_red_bypass_side_from_white_mask(
    dummy.data(), 10, 10, 10, 2, 2, 0, 4, 10.0, -1);
  assert(!s_zero_box.used_white_mask);
  assert(s_zero_box.side_sign == -1.0);

  const RedSideSelection s_out_of_bounds_y = select_red_bypass_side_from_white_mask(
    dummy.data(), 10, 10, 10, 2, 20, 4, 5, 10.0, 1);
  assert(!s_out_of_bounds_y.used_white_mask);
  assert(s_out_of_bounds_y.side_sign == 1.0);

  std::cout << "[PASS] test_invalid_inputs_fall_back_safely\n";
}

}  // namespace

int main()
{
  test_picks_wider_right_white_lane_even_when_left_img_margin_larger();
  test_picks_wider_left_white_lane_even_when_right_img_margin_larger();
  test_within_margin_uses_fallback_direction();
  test_invalid_inputs_fall_back_safely();
  std::cout << "All 4 red_side_selector unit tests passed.\n";
  return 0;
}
