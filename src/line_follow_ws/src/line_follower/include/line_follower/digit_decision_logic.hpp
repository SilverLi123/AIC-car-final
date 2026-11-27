#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <vector>

namespace line_follower
{
namespace decision
{

struct DigitCandidate
{
  int digit{0};
  double conf{0.0};
  double x_norm{0.5};
  double y_norm{0.5};
};

struct RobustDecisionConfig
{
  double conf_threshold{0.45};
  double trigger_y_min{0.32};
  double trigger_y_max{0.78};
  int vote_window_frames{4};
  int min_votes{2};
  double pair_x_sep_min{0.12};
  double post_yellow_window_sec{3.5};

  // Optional paired-slot anti-premature-trigger & anti-glare memory (opt-in: 0.0 / false = off).
  double pair_hold_sec{0.0};
  double pair_release_y{0.68};
  double slot_memory_sec{0.0};
  bool enforce_global_max_veto{false};
  double max_veto_min_conf{0.60};

  double turn_direction_sign{-1.0};
  double yaw_gain{1.6};
  double fork_turn_linear{0.20};
  double fork_max_yaw_rate{1.3};
  double fork_turn_duration{0.38};

  double straight_turn_linear{0.25};
  double straight_max_yaw_rate{0.55};
  double straight_turn_duration{0.25};

  double close_loop_x_tol{0.08};
  double close_loop_y_min{0.72};
  int missing_finish_frames{2};
};

struct TurnPlan
{
  bool trigger{false};
  int chosen_digit{-1};
  double target_x_norm{0.5};
  double target_y_norm{0.5};
  double total_conf{0.0};
  int votes{0};
  bool is_fork{false};
  double linear_x{0.0};
  double angular_z{0.0};
  double duration_sec{0.0};
};

class RobustDigitDecision
{
public:
  explicit RobustDigitDecision(const RobustDecisionConfig & cfg = RobustDecisionConfig{})
  : cfg_(cfg)
  {
  }

  void set_config(const RobustDecisionConfig & cfg)
  {
    cfg_ = cfg;
  }

  void reset_session()
  {
    window_.clear();
    slot_mem_[0] = RememberedSlot{};
    slot_mem_[1] = RememberedSlot{};
    solo_hold_active_ = false;
    solo_hold_start_sec_ = 0.0;
  }

  void reset_all()
  {
    reset_session();
    active_turn_ = false;
    target_digit_ = -1;
    target_x_norm_ = 0.5;
    last_seen_y_ = 0.0;
    missing_frames_ = 0;
  }

  void on_yellow_line(double now_sec)
  {
    has_yellow_ = true;
    last_yellow_time_sec_ = now_sec;
    reset_all();
  }

  TurnPlan process_frame(
    const std::vector<DigitCandidate> & raw_candidates,
    double centroid_x_norm,
    double now_sec)
  {
    std::vector<DigitCandidate> filtered;
    filtered.reserve(raw_candidates.size());
    for (const auto & c : raw_candidates) {
      if (c.digit < 1 || c.digit > 8) {
        continue;
      }
      if (c.conf < cfg_.conf_threshold) {
        continue;
      }
      if (c.y_norm < cfg_.trigger_y_min || c.y_norm > cfg_.trigger_y_max) {
        continue;
      }
      filtered.push_back(c);
    }

    const int max_win = std::max(1, cfg_.vote_window_frames);
    window_.push_back(std::move(filtered));
    while (static_cast<int>(window_.size()) > max_win) {
      window_.pop_front();
    }

    // Prune expired remembered slots.
    for (auto & s : slot_mem_) {
      if (s.valid && (cfg_.slot_memory_sec <= 0.0 ||
                      (now_sec - s.last_seen_sec) > cfg_.slot_memory_sec ||
                      now_sec < s.last_seen_sec))
      {
        s = RememberedSlot{};
      }
    }

    struct ObsWithFrame
    {
      DigitCandidate c;
      size_t frame_idx{0};
    };

    std::vector<ObsWithFrame> all_obs;
    const size_t num_frames = window_.size();
    for (size_t f = 0; f < num_frames; ++f) {
      for (const auto & item : window_[f]) {
        all_obs.push_back(ObsWithFrame{item, f});
      }
    }
    if (all_obs.empty()) {
      return TurnPlan{};
    }

    // Split observations in the sliding window into at most 2 spatial slots (left / right).
    std::sort(
      all_obs.begin(), all_obs.end(),
      [](const ObsWithFrame & a, const ObsWithFrame & b) {
        return a.c.x_norm < b.c.x_norm;
      });

    size_t split_idx = all_obs.size();
    double max_gap = 0.0;
    for (size_t i = 1; i < all_obs.size(); ++i) {
      const double gap = all_obs[i].c.x_norm - all_obs[i - 1].c.x_norm;
      if (gap >= cfg_.pair_x_sep_min && gap > max_gap) {
        max_gap = gap;
        split_idx = i;
      }
    }

    auto eval_slot = [&](size_t begin_idx, size_t end_idx) -> SlotWinner {
      SlotWinner res;
      if (begin_idx >= end_idx) {
        return res;
      }
      std::array<int, 9> votes{};
      std::array<double, 9> conf_sum{};
      std::array<double, 9> x_sum{};
      std::array<double, 9> y_sum{};
      std::array<double, 9> y_max{};

      // Count at most 1 vote per (frame, digit) inside a slot so duplicate boxes
      // in a single frame cannot satisfy min_votes without multi-frame evidence.
      std::vector<std::array<double, 9>> frame_best_conf(num_frames);
      std::vector<std::array<size_t, 9>> frame_best_idx(num_frames);
      for (auto & arr : frame_best_conf) {
        arr.fill(-1.0);
      }

      for (size_t i = begin_idx; i < end_idx; ++i) {
        const size_t f = all_obs[i].frame_idx;
        const int d = all_obs[i].c.digit;
        if (all_obs[i].c.conf > frame_best_conf[f][d]) {
          frame_best_conf[f][d] = all_obs[i].c.conf;
          frame_best_idx[f][d] = i;
        }
      }

      for (size_t f = 0; f < num_frames; ++f) {
        for (int d = 1; d <= 8; ++d) {
          if (frame_best_conf[f][d] < 0.0) {
            continue;
          }
          const DigitCandidate & c = all_obs[frame_best_idx[f][d]].c;
          votes[d]++;
          conf_sum[d] += c.conf;
          x_sum[d] += c.x_norm;
          y_sum[d] += c.y_norm;
          y_max[d] = std::max(y_max[d], c.y_norm);
        }
      }

      const int req_votes = std::max(1, cfg_.min_votes);
      for (int d = 1; d <= 8; ++d) {
        if (votes[d] < req_votes) {
          continue;
        }
        const bool better =
          !res.valid ||
          conf_sum[d] > res.total_conf + 1e-9 ||
          (std::abs(conf_sum[d] - res.total_conf) <= 1e-9 &&
           (votes[d] > res.votes || (votes[d] == res.votes && d > res.digit)));
        if (better) {
          res.valid = true;
          res.digit = d;
          res.votes = votes[d];
          res.total_conf = conf_sum[d];
          res.mean_x = x_sum[d] / static_cast<double>(votes[d]);
          res.mean_y = y_sum[d] / static_cast<double>(votes[d]);
          res.max_y = y_max[d];
        }
      }
      return res;
    };

    const SlotWinner cur_a = eval_slot(0, split_idx);
    const SlotWinner cur_b = eval_slot(split_idx, all_obs.size());

    auto pick_better = [](const SlotWinner & a, const SlotWinner & b) -> SlotWinner {
      if (!b.valid) {
        return a;
      }
      if (!a.valid) {
        return b;
      }
      if (a.digit != b.digit) {
        return (a.digit > b.digit) ? a : b;
      }
      return (a.total_conf >= b.total_conf) ? a : b;
    };

    auto remember_slot = [&](size_t idx, const SlotWinner & w) {
      slot_mem_[idx].valid = true;
      slot_mem_[idx].winner = w;
      slot_mem_[idx].last_seen_sec = now_sec;
    };

    int confirmed_slots = 0;
    SlotWinner chosen;

    if (cur_a.valid && cur_b.valid) {
      remember_slot(0, cur_a);
      remember_slot(1, cur_b);
      chosen = pick_better(cur_a, cur_b);
      confirmed_slots = 2;
    } else if (cur_a.valid || cur_b.valid) {
      const SlotWinner cur = cur_a.valid ? cur_a : cur_b;
      confirmed_slots = 1;
      chosen = cur;

      if (cfg_.slot_memory_sec > 0.0) {
        // Match against existing memory slot if within separation threshold; otherwise place by side.
        size_t target_idx = (cur.mean_x < 0.5) ? 0 : 1;
        double best_dist = 1e9;
        for (size_t idx = 0; idx < 2; ++idx) {
          if (slot_mem_[idx].valid) {
            const double d = std::abs(slot_mem_[idx].winner.mean_x - cur.mean_x);
            if (d < cfg_.pair_x_sep_min && d < best_dist) {
              best_dist = d;
              target_idx = idx;
            }
          }
        }
        const size_t partner_idx = 1 - target_idx;
        if (slot_mem_[partner_idx].valid &&
            std::abs(slot_mem_[partner_idx].winner.mean_x - cur.mean_x) >= cfg_.pair_x_sep_min)
        {
          // Keep the higher-valued or still-fresh remembered partner when glare hides it briefly.
          const SlotWinner partner = slot_mem_[partner_idx].winner;
          remember_slot(target_idx, cur);
          chosen = pick_better(cur, partner);
          confirmed_slots = 2;
        } else {
          remember_slot(target_idx, cur);
        }
      }
    } else {
      return TurnPlan{};
    }

    if (confirmed_slots == 1 && cfg_.pair_hold_sec > 0.0) {
      bool has_higher_pending = false;
      if (cfg_.enforce_global_max_veto) {
        const double veto_conf = std::max(cfg_.conf_threshold, cfg_.max_veto_min_conf);
        for (const auto & obs : all_obs) {
          if (obs.c.conf >= veto_conf &&
              obs.c.digit > chosen.digit &&
              std::abs(obs.c.x_norm - chosen.mean_x) >= cfg_.pair_x_sep_min)
          {
            has_higher_pending = true;
            break;
          }
        }
      }

      if (!solo_hold_active_) {
        solo_hold_active_ = true;
        solo_hold_start_sec_ = now_sec;
      }
      const double hold_elapsed = std::max(0.0, now_sec - solo_hold_start_sec_);
      if (hold_elapsed < cfg_.pair_hold_sec &&
          (chosen.max_y < cfg_.pair_release_y || has_higher_pending))
      {
        return TurnPlan{};
      }
    }
    solo_hold_active_ = false;

    const bool is_fork =
      has_yellow_ &&
      now_sec >= last_yellow_time_sec_ &&
      (now_sec - last_yellow_time_sec_) <= cfg_.post_yellow_window_sec;

    const double max_yaw = is_fork ? cfg_.fork_max_yaw_rate : cfg_.straight_max_yaw_rate;
    const double lin = is_fork ? cfg_.fork_turn_linear : cfg_.straight_turn_linear;
    const double dur = is_fork ? cfg_.fork_turn_duration : cfg_.straight_turn_duration;

    const double x_diff = chosen.mean_x - centroid_x_norm;
    const double yaw = std::clamp(
      cfg_.turn_direction_sign * x_diff * cfg_.yaw_gain, -max_yaw, max_yaw);

    if (std::abs(yaw) < 1e-6) {
      return TurnPlan{};
    }

    TurnPlan plan;
    plan.trigger = true;
    plan.chosen_digit = chosen.digit;
    plan.target_x_norm = chosen.mean_x;
    plan.target_y_norm = chosen.mean_y;
    plan.total_conf = chosen.total_conf;
    plan.votes = chosen.votes;
    plan.is_fork = is_fork;
    plan.linear_x = lin;
    plan.angular_z = yaw;
    plan.duration_sec = dur;

    active_turn_ = true;
    target_digit_ = chosen.digit;
    target_x_norm_ = chosen.mean_x;
    last_seen_y_ = chosen.max_y;
    missing_frames_ = 0;
    window_.clear();
    slot_mem_[0] = RememberedSlot{};
    slot_mem_[1] = RememberedSlot{};

    return plan;
  }

  bool should_finish_turn_early(const std::vector<DigitCandidate> & raw_candidates)
  {
    if (!active_turn_) {
      return false;
    }

    const DigitCandidate * matched = nullptr;
    double best_score = -1.0;

    for (const auto & c : raw_candidates) {
      if (c.digit != target_digit_ || c.conf < cfg_.conf_threshold) {
        continue;
      }
      const double score = c.conf - 0.5 * std::abs(c.x_norm - target_x_norm_);
      if (matched == nullptr || score > best_score) {
        matched = &c;
        best_score = score;
      }
    }

    if (matched != nullptr) {
      missing_frames_ = 0;
      target_x_norm_ = matched->x_norm;
      last_seen_y_ = std::max(last_seen_y_, matched->y_norm);

      if (std::abs(matched->x_norm - 0.5) <= cfg_.close_loop_x_tol &&
          matched->y_norm >= cfg_.close_loop_y_min)
      {
        active_turn_ = false;
        return true;
      }
      return false;
    }

    missing_frames_++;
    if (last_seen_y_ >= cfg_.close_loop_y_min &&
        missing_frames_ >= std::max(1, cfg_.missing_finish_frames))
    {
      active_turn_ = false;
      return true;
    }
    return false;
  }

private:
  struct SlotWinner
  {
    bool valid{false};
    int digit{-1};
    int votes{0};
    double total_conf{0.0};
    double mean_x{0.5};
    double mean_y{0.5};
    double max_y{0.0};
  };

  struct RememberedSlot
  {
    bool valid{false};
    SlotWinner winner{};
    double last_seen_sec{0.0};
  };

  RobustDecisionConfig cfg_;
  std::deque<std::vector<DigitCandidate>> window_;
  std::array<RememberedSlot, 2> slot_mem_{};
  bool solo_hold_active_{false};
  double solo_hold_start_sec_{0.0};

  bool has_yellow_{false};
  double last_yellow_time_sec_{0.0};

  bool active_turn_{false};
  int target_digit_{-1};
  double target_x_norm_{0.5};
  double last_seen_y_{0.0};
  int missing_frames_{0};
};

}  // namespace decision
}  // namespace line_follower
