#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "line_follower/red_freespace_carver.hpp"

using line_follower::BandChannelChoice;
using line_follower::CarvedRect;
using line_follower::RedCarveConfig;
using line_follower::RedObstacleBox;
using line_follower::compute_carved_rects;
using line_follower::select_widest_band_channel;

namespace
{

std::vector<uint8_t> make_white_road(int width, int height, int road_x0, int road_x1)
{
  std::vector<uint8_t> buf(static_cast<size_t>(width * height), 0);
  for (int y = 0; y < height; ++y) {
    for (int x = road_x0; x < road_x1; ++x) {
      buf[static_cast<size_t>(y * width + x)] = 255;
    }
  }
  return buf;
}

void apply_carve(std::vector<uint8_t> & buf, int width, const std::vector<CarvedRect> & rects)
{
  for (const auto & r : rects) {
    for (int y = r.y0; y < r.y1; ++y) {
      for (int x = r.x0; x < r.x1; ++x) {
        buf[static_cast<size_t>(y * width + x)] = 0;
      }
    }
  }
}

void test_left_side_cone_steers_down_right_corridor()
{
  // 640x160 ROI, road covers x=[120, 520) (center 320).
  // Red cone sits on left side of road: x=190..220, y=50..100 (y_rel=100/160=0.625).
  const int w = 640, h = 160;
  auto mask = make_white_road(w, h, 120, 520);
  RedCarveConfig cfg;
  const auto rects = compute_carved_rects({{190, 50, 30, 50, false}}, w, h, cfg);
  assert(rects.size() == 1);
  apply_carve(mask, w, rects);

  const int near_y0 = static_cast<int>(h * 0.62);
  const int near_y1 = static_cast<int>(h * 0.88);
  const BandChannelChoice ch = select_widest_band_channel(
    mask.data(), w, h, w, near_y0, near_y1, cfg.min_run_width, false, 0.0, cfg.side_hysteresis_ratio);

  assert(ch.valid);
  // Right corridor should start past x1 (padded cone) and end at 519, so center > 370 (right of 320).
  assert(ch.run_start >= rects[0].x1);
  assert(ch.center_x > 370.0 && ch.center_x < 520.0);
  std::cout << "[PASS] test_left_side_cone_steers_down_right_corridor\n";
}

void test_right_side_cone_steers_down_left_corridor()
{
  // Road covers x=[120, 520), cone sits on right side: x=420..450, y=50..100.
  const int w = 640, h = 160;
  auto mask = make_white_road(w, h, 120, 520);
  RedCarveConfig cfg;
  const auto rects = compute_carved_rects({{420, 50, 30, 50, false}}, w, h, cfg);
  assert(rects.size() == 1);
  apply_carve(mask, w, rects);

  const int near_y0 = static_cast<int>(h * 0.62);
  const int near_y1 = static_cast<int>(h * 0.88);
  const BandChannelChoice ch = select_widest_band_channel(
    mask.data(), w, h, w, near_y0, near_y1, cfg.min_run_width, false, 0.0, cfg.side_hysteresis_ratio);

  assert(ch.valid);
  assert(ch.run_end < rects[0].x0);
  assert(ch.center_x > 120.0 && ch.center_x < 270.0);
  std::cout << "[PASS] test_right_side_cone_steers_down_left_corridor\n";
}

void test_center_cone_hysteresis_prevents_flip_flop()
{
  // Road covers x=[100, 540). Cone is almost centered (left slightly wider initially, then right slightly wider).
  const int w = 640, h = 160;
  RedCarveConfig cfg;
  cfg.side_hysteresis_ratio = 0.80;

  // Frame 1: no previous commitment, cone slightly right of center -> left corridor slightly wider -> pick left.
  auto m1 = make_white_road(w, h, 100, 540);
  apply_carve(m1, w, compute_carved_rects({{325, 60, 20, 40, false}}, w, h, cfg));
  const BandChannelChoice c1 = select_widest_band_channel(
    m1.data(), w, h, w, 99, 140, cfg.min_run_width, false, 0.0, cfg.side_hysteresis_ratio);
  assert(c1.valid && c1.center_x < 250.0);

  // Frame 2: cone jitters slightly left so right corridor is now ~10px wider than left (~150 vs ~140, ratio > 0.80).
  // Hysteresis must keep the car committed to the left corridor instead of zig-zagging!
  auto m2 = make_white_road(w, h, 100, 540);
  apply_carve(m2, w, compute_carved_rects({{305, 60, 20, 40, false}}, w, h, cfg));
  const BandChannelChoice c2 = select_widest_band_channel(
    m2.data(), w, h, w, 99, 140, cfg.min_run_width, true, c1.center_x, cfg.side_hysteresis_ratio);
  assert(c2.valid && c2.center_x < 250.0);
  std::cout << "[PASS] test_center_cone_hysteresis_prevents_flip_flop\n";
}

void test_off_track_cone_does_not_disturb_road_center()
{
  // Road covers x=[180, 460) (center 319.5). Cone is far outside on left at x=10..30.
  const int w = 640, h = 160;
  auto mask = make_white_road(w, h, 180, 460);
  RedCarveConfig cfg;
  apply_carve(mask, w, compute_carved_rects({{10, 60, 20, 40, false}}, w, h, cfg));

  const BandChannelChoice ch = select_widest_band_channel(
    mask.data(), w, h, w, 99, 140, cfg.min_run_width, false, 0.0, cfg.side_hysteresis_ratio);
  assert(ch.valid);
  assert(std::abs(ch.center_x - 319.5) < 1e-6);
  std::cout << "[PASS] test_off_track_cone_does_not_disturb_road_center\n";
}

void test_downward_extension_projects_approaching_cone_into_near_band()
{
  // Cone is still in upper half of ROI: y=20..45 (y+h=45, ratio=45/160=0.281 >= 0.22),
  // while near band is y=[99, 140). Extension (+0.42*160 = +67px) reaches y=112 inside near band!
  const int w = 640, h = 160;
  auto mask = make_white_road(w, h, 120, 520);
  RedCarveConfig cfg;
  const auto rects = compute_carved_rects({{200, 20, 25, 25, false}}, w, h, cfg);
  assert(rects.size() == 1);
  assert(rects[0].y1 >= 110);
  apply_carve(mask, w, rects);

  const BandChannelChoice ch = select_widest_band_channel(
    mask.data(), w, h, w, 99, 140, cfg.min_run_width, false, 0.0, cfg.side_hysteresis_ratio);
  assert(ch.valid);
  assert(ch.center_x > 360.0);
  std::cout << "[PASS] test_downward_extension_projects_approaching_cone_into_near_band\n";
}

void test_curve_inner_cone_routes_around_remaining_curve_road()
{
  // Left curve: near road spans x=[40, 360) (normal center=200), far road spans x=[0, 260).
  // Cone sits on right/inner-right of road at x=220..250, y=70..110.
  // Carving leaves x=[40, x0) open so target shifts left within road instead of pushing right off track.
  const int w = 640, h = 160;
  auto mask = make_white_road(w, h, 40, 360);
  RedCarveConfig cfg;
  const auto rects = compute_carved_rects({{220, 70, 30, 40, false}}, w, h, cfg);
  assert(rects.size() == 1);
  apply_carve(mask, w, rects);

  const BandChannelChoice ch = select_widest_band_channel(
    mask.data(), w, h, w, 99, 140, cfg.min_run_width, false, 0.0, cfg.side_hysteresis_ratio);
  assert(ch.valid);
  assert(ch.run_start == 40);
  assert(ch.run_end < rects[0].x0);
  assert(ch.center_x < 150.0);
  std::cout << "[PASS] test_curve_inner_cone_routes_around_remaining_curve_road\n";
}

}  // namespace

int main()
{
  test_left_side_cone_steers_down_right_corridor();
  test_right_side_cone_steers_down_left_corridor();
  test_center_cone_hysteresis_prevents_flip_flop();
  test_off_track_cone_does_not_disturb_road_center();
  test_downward_extension_projects_approaching_cone_into_near_band();
  test_curve_inner_cone_routes_around_remaining_curve_road();
  std::cout << "All 6 red_freespace_carver unit tests passed.\n";
  return 0;
}
