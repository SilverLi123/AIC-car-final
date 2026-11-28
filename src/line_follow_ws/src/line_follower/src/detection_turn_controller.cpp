#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <ai_msgs/msg/perception_targets.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
constexpr double kDefaultConfThreshold = 0.35;
constexpr int kDefaultDetFrameWidth = 640;
constexpr int kDefaultDetFrameHeight = 480;
constexpr int kDefaultFrameWindow = 2;
constexpr double kDefaultTurnCooldown = 1.0;
constexpr double kDefaultTurnDuration = 0.46;
constexpr double kDefaultTurnLinear = 0.20;
constexpr double kDefaultYawGain = 1.65;
constexpr double kDefaultMaxYaw = 1.15;
constexpr double kDefaultYAlignTolerance = 0.025;
constexpr double kDefaultTurnDirectionSign = -1.0;  // 正值时 digit 在右转右，负值则相反
constexpr double kDefaultCentroidTimeout = 0.5;
constexpr const char * kDefaultLineCentroidTopic = "line_follower/centroid";
constexpr const char * kDefaultStateTopic = "line_follower/track_state";
constexpr const char * kDefaultStartTopic = "line_follower/start_cmd";
}  // namespace

class DetectionTurnController : public rclcpp::Node
{
public:
  DetectionTurnController()
  : Node("detection_turn_controller")
  {
    detection_topic_ = this->declare_parameter<std::string>("detection_topic", "hobot_dnn_detection");
    state_topic_ = this->declare_parameter<std::string>("state_topic", kDefaultStateTopic);
    start_topic_ = this->declare_parameter<std::string>("start_topic", kDefaultStartTopic);
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_detection");

    conf_threshold_ = this->declare_parameter<double>("confidence_threshold", kDefaultConfThreshold);
    det_frame_width_ = this->declare_parameter<int>("det_frame_width", kDefaultDetFrameWidth);
    det_frame_height_ = this->declare_parameter<int>("det_frame_height", kDefaultDetFrameHeight);
    frame_window_ = std::max<int>(1, static_cast<int>(this->declare_parameter<int>("frame_window", kDefaultFrameWindow)));
    turn_cooldown_ = this->declare_parameter<double>("turn_cooldown", kDefaultTurnCooldown);
    turn_duration_ = this->declare_parameter<double>("turn_duration", kDefaultTurnDuration);
    turn_linear_ = this->declare_parameter<double>("turn_linear", kDefaultTurnLinear);
    yaw_gain_ = this->declare_parameter<double>("yaw_gain", kDefaultYawGain);
    max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", kDefaultMaxYaw);
    y_align_tolerance_ = std::max(0.01, this->declare_parameter<double>("y_align_tolerance", kDefaultYAlignTolerance));
    turn_direction_sign_ = this->declare_parameter<double>("turn_direction_sign", kDefaultTurnDirectionSign);

    // Spatial-temporal multi-track voting & dual-badge sync parameters
    track_timeout_ = std::max(0.1, this->declare_parameter<double>("track_timeout", 0.45));
    dual_track_hold_time_ = std::max(0.0, this->declare_parameter<double>("dual_track_hold_time", 0.30));
    border_margin_x_ = std::clamp(this->declare_parameter<double>("border_margin_x", 0.16), 0.02, 0.30);
    min_track_y_ = std::clamp(this->declare_parameter<double>("min_track_y", 0.45), 0.0, 0.90);
    cross_below_margin_ = std::clamp(this->declare_parameter<double>("cross_below_margin", 0.14), 0.02, 0.30);
    border_early_margin_ = std::clamp(this->declare_parameter<double>("border_early_margin", 0.16), 0.01, 0.25);
    min_winner_share_ = std::clamp(this->declare_parameter<double>("min_winner_share", 0.50), 0.0, 1.0);
    min_winner_count_ = std::max<int>(1, static_cast<int>(this->declare_parameter<int>("min_winner_count", 2)));
    min_winner_conf_ = std::clamp(this->declare_parameter<double>("min_winner_conf", 0.55), 0.0, 1.0);
    min_normal_dy_travel_ = std::clamp(this->declare_parameter<double>("min_normal_dy_travel", 0.02), 0.0, 0.25);
    min_border_obs_ = std::max<int>(1, static_cast<int>(this->declare_parameter<int>("min_border_obs", 3)));
    min_turn_x_diff_ = std::clamp(this->declare_parameter<double>("min_turn_x_diff", 0.08), 0.0, 0.35);
    trigger_y_offset_ = std::clamp(this->declare_parameter<double>("trigger_y_offset", 0.0), -0.30, 0.30);
    // 兼容 16:28 v2 YAML 中的 enable_digit_centering 字段；与 enable_inlane_centering 严格解耦。
    enable_digit_centering_ = this->declare_parameter<bool>("enable_digit_centering", true);
    // 可选：接管转向时以车头中心 0.50 作为横向参考点，避免弯道减掉 road centroid 导致转向不足。
    use_image_center_ref_ = this->declare_parameter<bool>("use_image_center_ref", false);
    // 仅用于直道/车道内并排数字的闭环居中压数字实验；默认关闭，避免干扰正常开环转向。
    enable_inlane_centering_ = this->declare_parameter<bool>("enable_inlane_centering", false);
    pair_early_margin_ = std::clamp(this->declare_parameter<double>("pair_early_margin", 0.070), 0.0, 0.14);
    inlane_yaw_gain_ = std::clamp(this->declare_parameter<double>("inlane_yaw_gain", 3.60), 0.5, 6.0);
    inlane_turn_linear_ = std::clamp(this->declare_parameter<double>("inlane_turn_linear", 0.22), 0.08, 0.35);
    inlane_turn_duration_ = std::clamp(this->declare_parameter<double>("inlane_turn_duration", 1.75), 0.5, 3.0);
    inlane_rollover_hold_ = std::clamp(this->declare_parameter<double>("inlane_rollover_hold", 0.95), 0.2, 1.8);
    inlane_edge_guard_ = std::clamp(this->declare_parameter<double>("inlane_edge_guard", 0.28), 0.15, 0.45);
    require_start_signal_ = this->declare_parameter<bool>("require_start_signal", true);

    line_centroid_topic_ = this->declare_parameter<std::string>(
      "line_centroid_topic", kDefaultLineCentroidTopic);
    centroid_timeout_ = this->declare_parameter<double>("centroid_timeout", kDefaultCentroidTimeout);

    const auto now = this->now();
    turning_end_time_ = now;
    turn_cooldown_until_ = now;
    last_centroid_time_ = now - rclcpp::Duration::from_seconds(10.0);
    hold_start_time_ = now;
    active_target_last_seen_ = now;

    detection_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      detection_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&DetectionTurnController::detection_callback, this, std::placeholders::_1));

    centroid_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      line_centroid_topic_, rclcpp::QoS(10),
      std::bind(&DetectionTurnController::centroid_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(10),
      std::bind(&DetectionTurnController::state_callback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      start_topic_, rclcpp::QoS(10),
      std::bind(&DetectionTurnController::start_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&DetectionTurnController::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Detection turn controller (multi-track) listening '%s', start '%s', centroid '%s', "
      "publishing '%s' (conf>=%.2f, win_conf>=%.2f, min_obs=%d, dy>=%.2f, y_off=%.2f, dur=%.2fs, hold=%.2fs)",
      detection_topic_.c_str(), start_topic_.c_str(), line_centroid_topic_.c_str(),
      cmd_vel_topic_.c_str(), conf_threshold_, min_winner_conf_, frame_window_,
      min_normal_dy_travel_, trigger_y_offset_, turn_duration_, dual_track_hold_time_);
  }

private:
  struct TrackObs
  {
    rclcpp::Time stamp;
    int digit;
    double conf;
    double cx;
    double cy;
  };

  struct DigitTrack
  {
    int id{0};
    double cx{0.0};
    double cy{0.0};
    double first_cy{0.0};
    double max_cy{0.0};
    rclcpp::Time first_t;
    rclcpp::Time last_t;
    std::deque<TrackObs> obs;
  };

  struct TrackEval
  {
    int digit{-1};
    double max_conf{0.0};
    int label_count{0};
    double share{0.0};
    double cx{0.5};
    double cy{0.5};
    bool crossed{false};
    bool border_ready{false};
    bool approaching{false};
  };

  static int parse_digit_label(const std::string & label)
  {
    if (label.empty()) {
      return -1;
    }
    size_t start = (label[0] == 'd' || label[0] == 'D') ? 1 : 0;
    if (start >= label.size()) {
      return -1;
    }
    for (size_t i = start; i < label.size(); ++i) {
      if (!std::isdigit(static_cast<unsigned char>(label[i]))) {
        return -1;
      }
    }
    int value = -1;
    try {
      value = std::stoi(label.substr(start));
    } catch (...) {
      return -1;
    }
    if (value >= 1 && value <= 8) {
      return value;
    }
    return -1;
  }

  void clear_tracks()
  {
    tracks_.clear();
    hold_active_ = false;
  }

  void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const bool new_state = msg->data;
    if (new_state == start_enabled_) {
      return;
    }
    start_enabled_ = new_state;
    clear_tracks();
    if (!start_enabled_) {
      turning_ = false;
      inlane_centering_active_ = false;
      inlane_rollover_active_ = false;
      yellow_stopped_ = false;
      turn_cooldown_until_ = this->now();
    }
  }

  void centroid_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    last_centroid_norm_x_ = std::clamp(static_cast<double>(msg->x), 0.0, 1.0);
    last_centroid_norm_y_ = std::clamp(static_cast<double>(msg->y), 0.0, 1.0);
    last_centroid_time_ = this->now();
  }

  void state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const auto & s = msg->data;
    if (s == "yellow_line_detected") {
      // 黄线触发：闭锁并在黄停期间清空数字轨迹，防止停车静止阶段积累远处误检或终点 END 字母。
      yellow_stopped_ = true;
      clear_tracks();
      turning_ = false;
      inlane_centering_active_ = false;
      inlane_rollover_active_ = false;
      turn_cooldown_until_ = this->now();
    } else if (s == "tracking") {
      if (yellow_stopped_) {
        yellow_stopped_ = false;
        clear_tracks();
      }
    }
  }

  static TrackEval evaluate_track(
    const DigitTrack & tr,
    double y_ref,
    double y_tol,
    double cross_below,
    double border_margin_x,
    double border_early,
    double min_normal_dy_travel,
    int min_border_obs)
  {
    std::array<double, 9> scores{};
    std::array<int, 9> counts{};
    std::array<double, 9> max_confs{};
    double total_w = 0.0;

    const size_t n = tr.obs.size();
    for (size_t i = 0; i < n; ++i) {
      const auto & o = tr.obs[i];
      if (o.digit < 1 || o.digit > 8) {
        continue;
      }
      // 3.6MB BPU YOLOv8 在中距离清晰区（cy=0.52~0.70，完整入框）准确率 85~93%；
      // 贴近底边（cy>0.74）或左右边缘被裁切（cx<0.10 或 cx>0.90）时严重透视畸变易跳类。
      // 因此采用清晰区高斯钟形权重 + 裁边降权 + 温和时间衰减，避免单帧近距误检覆盖清晰期投票。
      const double age = static_cast<double>(n - 1 - i);
      const double w_rec = std::pow(0.92, age);
      const double dy_gold = (o.cy - 0.62) / 0.12;
      const double w_zone = 0.30 + 0.70 * std::exp(-0.5 * dy_gold * dy_gold);
      const double edge_dist = std::min(o.cx, 1.0 - o.cx);
      const double w_edge = (edge_dist < 0.10) ? (0.45 + 5.5 * edge_dist) : 1.0;
      const double w = w_rec * w_zone * w_edge * std::pow(std::max(0.05, o.conf), 2.0);
      scores[o.digit] += w;
      counts[o.digit] += 1;
      max_confs[o.digit] = std::max(max_confs[o.digit], o.conf);
      total_w += w;
    }

    TrackEval ev{};
    int best_d = -1;
    double best_score = -1.0;
    for (int d = 1; d <= 8; ++d) {
      if (scores[d] > best_score ||
          (std::abs(scores[d] - best_score) < 1e-9 && d > best_d))
      {
        if (counts[d] > 0) {
          best_score = scores[d];
          best_d = d;
        }
      }
    }
    if (best_d < 0) {
      return ev;
    }

    ev.digit = best_d;
    ev.max_conf = max_confs[best_d];
    ev.label_count = counts[best_d];
    ev.share = (total_w > 1e-9) ? (best_score / total_w) : 0.0;
    ev.cx = tr.cx;
    ev.cy = tr.cy;

    // 真实地面数字从上方进入触发带（放宽至 +0.08 兼容岔路口右侧大角度晚入框数字）
    const bool valid_approach = (tr.first_cy <= y_ref + 0.08);
    const bool near_border = (tr.cx <= border_margin_x || tr.cx >= (1.0 - border_margin_x));
    const double dy_travel = tr.max_cy - tr.first_cy;

    ev.crossed =
      valid_approach &&
      (tr.cy >= y_ref - y_tol) &&
      (tr.cy <= y_ref + cross_below) &&
      (dy_travel >= min_normal_dy_travel);
    ev.border_ready =
      valid_approach &&
      near_border &&
      (static_cast<int>(tr.obs.size()) >= min_border_obs) &&
      (tr.cy >= y_ref - border_early) &&
      (dy_travel >= 0.005);
    ev.approaching =
      valid_approach &&
      (tr.cy >= y_ref - 0.12) &&
      (tr.cy < y_ref - y_tol);
    return ev;
  }

  void detection_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const rclcpp::Time now = this->now();

    if ((require_start_signal_ && !start_enabled_) || yellow_stopped_) {
      clear_tracks();
      return;
    }

    if (turning_) {
      // 仅对车道内并排数字启用闭环居中 + 滚压直行（岔口 border-early 保持原版开环不变）
      if (inlane_centering_active_ && now < turning_end_time_) {
        const double fw = det_frame_width_ > 0 ? static_cast<double>(det_frame_width_) : 640.0;
        const double fh = det_frame_height_ > 0 ? static_cast<double>(det_frame_height_) : 480.0;

        // 边界安全护栏：若车身已接近黑线边缘（道路质心偏移超限），立即回正角速度防压线
        const double lane_offset = last_centroid_norm_x_ - 0.50;
        const bool near_edge_left = (active_turn_dir_ > 0.0 && lane_offset > inlane_edge_guard_);
        const bool near_edge_right = (active_turn_dir_ < 0.0 && lane_offset < -inlane_edge_guard_);
        if (near_edge_left || near_edge_right) {
          turning_cmd_.angular.z = 0.0;
          if (inlane_rollover_active_) {
            turning_ = false;
            inlane_centering_active_ = false;
            inlane_rollover_active_ = false;
            return;
          }
          cmd_pub_->publish(turning_cmd_);
          return;
        }

        if (inlane_rollover_active_) {
          return;
        }

        // 在当前帧中按空间最近邻追踪已选定的目标数字（不切换目标）
        double best_dist = 0.22;
        double live_cx = -1.0;
        double live_cy = 0.0;
        for (const auto & target : msg->targets) {
          for (const auto & roi : target.rois) {
            if (roi.rect.width <= 0 || roi.rect.height <= 0 || roi.confidence < conf_threshold_) {
              continue;
            }
            const int d = parse_digit_label(roi.type);
            if (d <= 0) {
              continue;
            }
            const double cx = std::clamp(
              (static_cast<double>(roi.rect.x_offset) + static_cast<double>(roi.rect.width) * 0.5) / fw,
              0.0, 1.0);
            const double cy = std::clamp(
              (static_cast<double>(roi.rect.y_offset) + static_cast<double>(roi.rect.height) * 0.5) / fh,
              0.0, 1.0);
            if (cy < 0.55 || cy > 0.94) {
              continue;
            }
            const double dist = std::abs(cx - active_target_cx_);
            if (dist < best_dist) {
              best_dist = dist;
              live_cx = cx;
              live_cy = cy;
            }
          }
        }

        if (live_cx >= 0.0) {
          active_target_cx_ = live_cx;
          active_target_cy_ = std::max(active_target_cy_, live_cy);
          active_target_last_seen_ = now;
          const double err_center = live_cx - 0.50;
          if (live_cy >= 0.84 && std::abs(err_center) < 0.08) {
            // 目标数字已居中并到达画面底边：转入直行滚压阶段，让前后轴完整压过数字中心
            inlane_rollover_active_ = true;
            turning_cmd_.angular.z = 0.0;
            const auto roll_end = now + rclcpp::Duration::from_seconds(inlane_rollover_hold_);
            if (roll_end < turning_end_time_) {
              turning_end_time_ = roll_end;
              turn_cooldown_until_ = turning_end_time_ + rclcpp::Duration::from_seconds(turn_cooldown_);
            }
            cmd_pub_->publish(turning_cmd_);
            RCLCPP_INFO(
              get_logger(),
              "[digit rollover] centered at cx=%.2f cy=%.2f -> straight hold %.2fs",
              live_cx, live_cy, inlane_rollover_hold_);
            return;
          }
          turning_cmd_.angular.z = std::clamp(
            turn_direction_sign_ * err_center * inlane_yaw_gain_, -max_yaw_rate_, max_yaw_rate_);
          cmd_pub_->publish(turning_cmd_);
        } else if ((active_target_cy_ >= 0.76 && (now - active_target_last_seen_).seconds() >= 0.09) ||
                   (now - active_target_last_seen_).seconds() >= 0.18)
        {
          const double hold_sec = (active_target_cy_ >= 0.76)
            ? inlane_rollover_hold_
            : std::min(inlane_rollover_hold_, 0.55);
          inlane_rollover_active_ = true;
          turning_cmd_.angular.z = 0.0;
          const auto roll_end = now + rclcpp::Duration::from_seconds(hold_sec);
          if (roll_end < turning_end_time_) {
            turning_end_time_ = roll_end;
            turn_cooldown_until_ = turning_end_time_ + rclcpp::Duration::from_seconds(turn_cooldown_);
          }
          cmd_pub_->publish(turning_cmd_);
          RCLCPP_INFO(
            get_logger(),
            "[digit rollover] exited/lost (last cx=%.2f cy=%.2f) -> straight hold %.2fs",
            active_target_cx_, active_target_cy_, hold_sec);
        }
      }
      return;
    }
    if (now < turn_cooldown_until_) {
      return;
    }

    const bool centroid_valid =
      (now - last_centroid_time_).seconds() <= centroid_timeout_;
    if (!centroid_valid) {
      return;
    }

    // 清理超时轨迹
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const DigitTrack & tr) {
          return (now - tr.last_t).seconds() > track_timeout_;
        }),
      tracks_.end());

    const double frame_w = det_frame_width_ > 0 ? static_cast<double>(det_frame_width_) : 640.0;
    const double frame_h = det_frame_height_ > 0 ? static_cast<double>(det_frame_height_) : 480.0;

    // 空间最近邻关联：将本帧检测挂载到已有物理数字轨迹或新建轨迹
    std::vector<bool> track_used(tracks_.size(), false);
    for (const auto & target : msg->targets) {
      for (const auto & roi : target.rois) {
        if (roi.rect.width <= 0 || roi.rect.height <= 0 || roi.confidence < conf_threshold_) {
          continue;
        }
        const int digit = parse_digit_label(roi.type);
        if (digit < 0) {
          continue;
        }
        const double cx = std::clamp(
          (static_cast<double>(roi.rect.x_offset) + static_cast<double>(roi.rect.width) * 0.5) / frame_w,
          0.0, 1.0);
        const double cy = std::clamp(
          (static_cast<double>(roi.rect.y_offset) + static_cast<double>(roi.rect.height) * 0.5) / frame_h,
          0.0, 1.0);
        if (cy < min_track_y_) {
          continue;
        }

        int best_ti = -1;
        double best_dist = 1e9;
        for (size_t ti = 0; ti < tracks_.size(); ++ti) {
          if (ti < track_used.size() && track_used[ti]) {
            continue;
          }
          const double dx = std::abs(cx - tracks_[ti].cx);
          const double dy = cy - tracks_[ti].cy;
          if (dx <= 0.13 && dy >= -0.05 && dy <= 0.16) {
            const double dist = std::hypot(dx, dy);
            if (dist < best_dist) {
              best_dist = dist;
              best_ti = static_cast<int>(ti);
            }
          }
        }

        if (best_ti >= 0) {
          auto & tr = tracks_[static_cast<size_t>(best_ti)];
          if (static_cast<size_t>(best_ti) < track_used.size()) {
            track_used[static_cast<size_t>(best_ti)] = true;
          }
          tr.cx = cx;
          tr.cy = cy;
          tr.last_t = now;
          tr.max_cy = std::max(tr.max_cy, cy);
          tr.obs.push_back(TrackObs{now, digit, static_cast<double>(roi.confidence), cx, cy});
          while (tr.obs.size() > 14) {
            tr.obs.pop_front();
          }
        } else {
          DigitTrack tr;
          tr.id = next_track_id_++;
          tr.cx = cx;
          tr.cy = cy;
          tr.first_cy = cy;
          tr.max_cy = cy;
          tr.first_t = now;
          tr.last_t = now;
          tr.obs.push_back(TrackObs{now, digit, static_cast<double>(roi.confidence), cx, cy});
          tracks_.push_back(std::move(tr));
        }
      }
    }

    if (tracks_.empty()) {
      hold_active_ = false;
      return;
    }

    const double y_ref = std::clamp(last_centroid_norm_y_ + trigger_y_offset_, 0.20, 0.92);
    const double eff_y_tol = std::max(0.025, y_align_tolerance_);

    std::vector<TrackEval> ready;
    std::vector<TrackEval> pair_early_candidates;
    bool has_approaching_other = false;
    double max_ready_y = 0.0;
    bool any_border_urgent = false;

    for (const auto & tr : tracks_) {
      if (static_cast<int>(tr.obs.size()) < frame_window_) {
        continue;
      }
      const TrackEval ev = evaluate_track(
        tr, y_ref, eff_y_tol, cross_below_margin_, border_margin_x_, border_early_margin_,
        min_normal_dy_travel_, min_border_obs_);
      if (ev.digit < 0) {
        continue;
      }
      if (ev.crossed || ev.border_ready) {
        ready.push_back(ev);
        max_ready_y = std::max(max_ready_y, ev.cy);
        if (ev.border_ready && !ev.crossed) {
          any_border_urgent = true;
        }
      } else if (ev.approaching) {
        has_approaching_other = true;
      }

      // 车道内并排双数字提前对中候选：非边缘目标、高共识 (>=4帧 & 占比>=0.70)、进入提前触发带
      if (enable_inlane_centering_ && !ev.border_ready &&
          ev.label_count >= std::max(4, min_winner_count_) &&
          ev.share >= 0.70 &&
          ev.cy >= (y_ref - pair_early_margin_) &&
          ev.cy <= (y_ref + cross_below_margin_) &&
          tr.first_cy <= y_ref + 0.08)
      {
        pair_early_candidates.push_back(ev);
      }
    }

    if (ready.empty() && pair_early_candidates.size() >= 2) {
      double min_cy = 1.0;
      double max_cy = 0.0;
      for (const auto & ev : pair_early_candidates) {
        min_cy = std::min(min_cy, ev.cy);
        max_cy = std::max(max_cy, ev.cy);
      }
      if ((max_cy - min_cy) <= 0.05) {
        ready = pair_early_candidates;
        max_ready_y = max_cy;
        has_approaching_other = false;
      }
    }

    if (ready.empty()) {
      hold_active_ = false;
      return;
    }

    // 双目标短时等待：若一侧数字先越线而另一侧轨迹紧随其后，短等最多 dual_track_hold_time_。
    if (has_approaching_other && !any_border_urgent && dual_track_hold_time_ > 0.0) {
      if (!hold_active_) {
        hold_active_ = true;
        hold_start_time_ = now;
      }
      if ((now - hold_start_time_).seconds() < dual_track_hold_time_ &&
          max_ready_y < (y_ref + 0.055))
      {
        return;
      }
    }

    // 过滤低共识/低置信度噪声轨迹（防止单帧误检或反光抖动误触发转向）
    std::vector<TrackEval> credible;
    for (const auto & ev : ready) {
      if (ev.label_count >= min_winner_count_ && ev.share >= min_winner_share_ &&
          ev.max_conf >= min_winner_conf_)
      {
        credible.push_back(ev);
      } else if (ev.border_ready && ev.max_conf >= std::max(0.60, min_winner_conf_) &&
                 ev.share >= 0.45)
      {
        // 宽岔口边缘快速滑出目标允许高置信度放行
        credible.push_back(ev);
      }
    }
    if (credible.empty()) {
      return;
    }

    const auto winner = *std::max_element(
      credible.begin(), credible.end(),
      [](const TrackEval & a, const TrackEval & b) {
        if (a.digit != b.digit) {
          return a.digit < b.digit;
        }
        if (std::abs(a.share - b.share) > 1e-6) {
          return a.share < b.share;
        }
        return a.max_conf < b.max_conf;
      });

    // 若胜出数字基本就在当前车头正前方（|x_diff| < min_turn_x_diff_），不抢占巡线通道。
    const double win_x_diff = winner.cx - last_centroid_norm_x_;
    if (std::abs(win_x_diff) < min_turn_x_diff_) {
      RCLCPP_INFO(
        get_logger(),
        "[digit skip] best:%d conf:%.2f share:%.2f |x_diff|=%.2f < %.2f (keep line follower)",
        winner.digit, winner.max_conf, winner.share, std::abs(win_x_diff), min_turn_x_diff_);
      turn_cooldown_until_ = now + rclcpp::Duration::from_seconds(0.35);
      clear_tracks();
      return;
    }

    trigger_turn(now, winner, y_ref, static_cast<int>(credible.size()));
    clear_tracks();
  }

  void trigger_turn(const rclcpp::Time & now, const TrackEval & winner, double y_ref, int ready_count)
  {
    const bool use_inlane_centering =
      enable_inlane_centering_ &&
      !winner.border_ready &&
      (ready_count >= 2) &&
      (std::abs(winner.cx - 0.50) <= 0.28);
    const double ref_x = (use_inlane_centering || use_image_center_ref_) ? 0.50 : last_centroid_norm_x_;
    const double eff_gain = use_inlane_centering ? inlane_yaw_gain_ : yaw_gain_;
    const double eff_lin = use_inlane_centering ? inlane_turn_linear_ : turn_linear_;
    const double eff_dur = use_inlane_centering ? inlane_turn_duration_ : turn_duration_;

    const double x_diff = winner.cx - ref_x;
    const double y_diff = winner.cy - y_ref;
    const double yaw = std::clamp(
      turn_direction_sign_ * x_diff * eff_gain, -max_yaw_rate_, max_yaw_rate_);
    if (std::abs(yaw) < 1e-4) {
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = eff_lin;
    cmd.angular.z = yaw;

    turning_cmd_ = cmd;
    turning_end_time_ = now + rclcpp::Duration::from_seconds(eff_dur);
    turn_cooldown_until_ = turning_end_time_ + rclcpp::Duration::from_seconds(turn_cooldown_);
    turning_ = true;
    inlane_centering_active_ = use_inlane_centering;
    inlane_rollover_active_ = false;
    active_target_cx_ = winner.cx;
    active_target_cy_ = winner.cy;
    active_target_last_seen_ = now;
    active_turn_dir_ = (yaw >= 0.0) ? 1.0 : -1.0;

    // 立即发布首帧转向指令，不等待下一个 50ms 定时器周期
    cmd_pub_->publish(turning_cmd_);

    RCLCPP_INFO(
      get_logger(),
      "[digit turn] best:%d conf:%.2f share:%.2f x_diff:%.2f y_diff:%.2f dur:%.2fs "
      "lin:%.2f ang:%.2f (tracks:%d%s)",
      winner.digit, winner.max_conf, winner.share, x_diff, y_diff, eff_dur,
      cmd.linear.x, cmd.angular.z, ready_count,
      use_inlane_centering ? ", inlane-center" :
      ((winner.border_ready && !winner.crossed) ? ", border-early" : ""));
  }

  void on_timer()
  {
    if (!turning_) {
      return;
    }
    if ((require_start_signal_ && !start_enabled_) || yellow_stopped_ ||
        this->now() >= turning_end_time_)
    {
      turning_ = false;
      inlane_centering_active_ = false;
      inlane_rollover_active_ = false;
      return;
    }
    cmd_pub_->publish(turning_cmd_);
  }

  // Params
  std::string detection_topic_;
  std::string state_topic_;
  std::string start_topic_;
  std::string cmd_vel_topic_;
  std::string line_centroid_topic_;
  double conf_threshold_{kDefaultConfThreshold};
  int det_frame_width_{kDefaultDetFrameWidth};
  int det_frame_height_{kDefaultDetFrameHeight};
  int frame_window_{kDefaultFrameWindow};
  double turn_cooldown_{kDefaultTurnCooldown};
  double turn_duration_{kDefaultTurnDuration};
  double turn_linear_{kDefaultTurnLinear};
  double yaw_gain_{kDefaultYawGain};
  double max_yaw_rate_{kDefaultMaxYaw};
  double y_align_tolerance_{kDefaultYAlignTolerance};
  double turn_direction_sign_{kDefaultTurnDirectionSign};
  double centroid_timeout_{kDefaultCentroidTimeout};
  double track_timeout_{0.45};
  double dual_track_hold_time_{0.30};
  double border_margin_x_{0.16};
  double min_track_y_{0.45};
  double cross_below_margin_{0.14};
  double border_early_margin_{0.16};
  double min_winner_share_{0.50};
  int min_winner_count_{2};
  double min_winner_conf_{0.55};
  double min_normal_dy_travel_{0.02};
  int min_border_obs_{3};
  double min_turn_x_diff_{0.08};
  double trigger_y_offset_{0.0};
  bool enable_digit_centering_{true};
  bool use_image_center_ref_{false};
  bool enable_inlane_centering_{false};
  double pair_early_margin_{0.070};
  double inlane_yaw_gain_{3.60};
  double inlane_turn_linear_{0.22};
  double inlane_turn_duration_{1.75};
  double inlane_rollover_hold_{0.95};
  double inlane_edge_guard_{0.28};
  bool require_start_signal_{true};

  // State
  bool start_enabled_{false};
  bool yellow_stopped_{false};
  bool turning_{false};
  bool inlane_centering_active_{false};
  bool inlane_rollover_active_{false};
  double active_target_cx_{0.5};
  double active_target_cy_{0.5};
  rclcpp::Time active_target_last_seen_{0, 0, RCL_ROS_TIME};
  double active_turn_dir_{0.0};
  geometry_msgs::msg::Twist turning_cmd_;
  rclcpp::Time turning_end_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time turn_cooldown_until_{0, 0, RCL_ROS_TIME};

  int next_track_id_{1};
  std::vector<DigitTrack> tracks_;
  bool hold_active_{false};
  rclcpp::Time hold_start_time_{0, 0, RCL_ROS_TIME};

  double last_centroid_norm_x_{0.5};
  double last_centroid_norm_y_{0.6458};
  rclcpp::Time last_centroid_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr detection_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr centroid_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionTurnController>());
  rclcpp::shutdown();
  return 0;
}
