#include "line_follower/line_follower_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/bool.hpp>

namespace
{

struct CurveAssistRuntime
{
  bool reverse_curve_active{false};
  int reverse_confirm_count{0};
  int reverse_clear_count{0};
  bool near_valid{false};
  bool far_valid{false};
  double near_error{0.0};
  double far_error{0.0};
  double curve_level{0.0};
};

CurveAssistRuntime g_curve_assist;

// 红色目标运行时信息。
// 放在 cpp 内部，避免为了这一版修改 line_follower_node.hpp。
struct RedAvoidRuntime
{
  bool valid{false};

  // 横向红色停车线：与红锥分类处理，但两者都需要避障。
  bool red_line_detected{false};
  bool is_red_line{false};
  cv::Rect red_line_bbox_img{};

  // 当前被当作红锥使用的目标。
  cv::Rect bbox_img{};
  cv::Point2d center_img{};
  cv::Point2d bottom_img{};

  double aspect_ratio{0.0};
  double contour_area_ratio{0.0};
  double bottom_ratio{0.0};

  bool in_corridor{false};
  double risk{0.0};
  bool very_close{false};

  // 红障绕行方向锁存（跨帧保留，不随 reset_frame() 清零）。
  bool dir_locked{false};
  double locked_side_sign{0.0};
  int dir_clear_frames{0};

  void reset_frame()
  {
    valid = false;
    red_line_detected = false;
    is_red_line = false;
    red_line_bbox_img = cv::Rect{};
    bbox_img = cv::Rect{};
    center_img = cv::Point2d{};
    bottom_img = cv::Point2d{};
    aspect_ratio = 0.0;
    contour_area_ratio = 0.0;
    bottom_ratio = 0.0;
    in_corridor = false;
    risk = 0.0;
    very_close = false;
  }

  void reset_lock()
  {
    dir_locked = false;
    locked_side_sign = 0.0;
    dir_clear_frames = 0;
  }

  void decay_lock(bool keep_enabled, int clear_frames_req)
  {
    if (!keep_enabled) {
      reset_lock();
      return;
    }
    if (dir_locked) {
      dir_clear_frames++;
      if (dir_clear_frames >= std::max(1, clear_frames_req)) {
        reset_lock();
      }
    }
  }
};

RedAvoidRuntime g_red_avoid;

// 运行过程中短暂丢线时，保持一个低速前进和上一帧转向，
// 避免除黄色停车线之外的情况把车停住。
struct MotionRuntime
{
  bool valid{false};
  double last_linear{0.08};
  double last_yaw{0.0};
};

MotionRuntime g_motion;

}  // namespace

namespace line_follower
{

LineFollowerNode::LineFollowerNode()
: Node("line_follower_node")
{
  declare_parameters();
  setup_interfaces();

  const int kernel_size = std::max<int>(1, morph_kernel_);
  morph_element_ = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  last_detection_time_ = this->now();
  yellow_line_detection_time_ = this->now();
  last_state_ = TrackState::LOST;
  yellow_line_stop_active_ = false;
  yellow_line_detection_count_ = 0;
  yellow_line_latched_ = false;
  yellow_clear_frames_ = 0;
  permanent_stop_ = false;
  start_enabled_ = false;
  red_left_votes_ = 0;
  red_right_votes_ = 0;
  red_turning_ = false;
  red_turn_end_time_ = this->now();
  last_red_overlay_valid_ = false;

  RCLCPP_INFO(
    this->get_logger(),
    "Line follower subscribing to '%s', publishing cmd_vel on '%s', start_topic: '%s'",
    image_topic_.c_str(), cmd_vel_topic_.c_str(), start_topic_.c_str());

  if (this->get_parameter("enable_curve_preview_steering").as_bool()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Fork curve-preview steering enabled: max_far_weight=%.2f, yaw_slew=%.2f rad/s",
      this->get_parameter("curve_preview_max_far_weight").as_double(),
      this->get_parameter("curve_preview_yaw_slew_rate").as_double());
  }
}

void LineFollowerNode::declare_parameters()
{
  this->declare_parameter("image_topic", "/ascamera/camera_publisher/rgb/image0");
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter("debug_image_topic", "line_follower/debug_image");
  this->declare_parameter("line_state_topic", "line_follower/track_state");
  this->declare_parameter("start_topic", "line_follower/start_cmd");
  this->declare_parameter("stop_cmd_topic", "/cmd_vel_stop");
  this->declare_parameter("line_centroid_topic", "line_follower/centroid");

  this->declare_parameter("target_speed", 0.6);
  this->declare_parameter("max_speed", 1.0);
  this->declare_parameter("min_speed", 0.0);
  this->declare_parameter("max_yaw_rate", 1.2);
  this->declare_parameter("kp", 1.8);

  this->declare_parameter("roi_top_ratio", 0.5);
  this->declare_parameter("roi_bottom_ratio", 0.05);
  this->declare_parameter("gaussian_kernel", 5);
  this->declare_parameter("binary_threshold", 160.0);
  this->declare_parameter("morph_kernel", 5);
  this->declare_parameter("morph_iterations", 2);
  this->declare_parameter("min_contour_area", 2000.0);
  this->declare_parameter("invert_binary", false);
  this->declare_parameter("use_white_filter", false);
  this->declare_parameter("white_h_low", 0);
  this->declare_parameter("white_h_high", 180);
  this->declare_parameter("white_s_max", 60);
  this->declare_parameter("white_v_min", 180);

  this->declare_parameter("use_yellow_filter", false);
  this->declare_parameter("yellow_h_low", 20);
  this->declare_parameter("yellow_h_high", 40);
  this->declare_parameter("yellow_s_min", 80);
  this->declare_parameter("yellow_v_min", 120);
  this->declare_parameter("yellow_area_ratio_threshold", 0.1);
  this->declare_parameter("yellow_stop_duration", 1.0);
  this->declare_parameter("yellow_permanent_count", 5);
  this->declare_parameter("yellow_clear_frames", 10);
  // 实验功能（默认关闭）：黄线横向几何过滤与 /odom 停稳闭环确认。
  this->declare_parameter("enable_yellow_geo_filter", false);
  this->declare_parameter("yellow_min_width_ratio", 0.25);
  this->declare_parameter("yellow_min_aspect_ratio", 2.5);
  this->declare_parameter("enable_yellow_odom_stop_confirm", false);
  this->declare_parameter("yellow_still_speed_mps", 0.03);
  this->declare_parameter("yellow_still_hold_sec", 0.15);
  this->declare_parameter("yellow_stop_max_duration", 1.5);
  this->declare_parameter("enable_reverse_brake", false);
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("reverse_speed", 0.0);
  this->declare_parameter("reverse_max_duration", 0.0);
  this->declare_parameter("reverse_cutoff_speed", 0.10);
  this->declare_parameter("reverse_abort_speed", 0.01);
  this->declare_parameter("reverse_odom_timeout", 0.15);
  this->declare_parameter("enable_intersection_probe", false);
  this->declare_parameter("intersection_band_height", 50);
  this->declare_parameter("intersection_dark_ratio_threshold", 0.9);
  this->declare_parameter("intersection_turn_speed", 0.2);
  this->declare_parameter("intersection_turn_yaw_rate", 0.8);
  this->declare_parameter("intersection_turn_duration", 1.0);
  this->declare_parameter("intersection_cooldown", 2.0);
  // 红色避障
  this->declare_parameter("enable_red_filter", false);
  this->declare_parameter("red_h_low_1", 0);
  this->declare_parameter("red_h_high_1", 10);
  this->declare_parameter("red_h_low_2", 160);
  this->declare_parameter("red_h_high_2", 180);
  this->declare_parameter("red_s_min", 100);
  this->declare_parameter("red_v_min", 80);
  this->declare_parameter("red_area_ratio_threshold", 0.005);
  this->declare_parameter("red_turn_yaw", 0.6);
  this->declare_parameter("red_turn_linear", 0.0);
  this->declare_parameter("red_turn_duration", 1.0);
  this->declare_parameter("red_vote_frames", 8);
  this->declare_parameter("red_divider_angle_deg", 0.0);

  // 连续弯辅助：默认关闭，关闭时完全沿用原 70% near + 30% far 逻辑。
  this->declare_parameter("enable_continuous_curve_assist", false);
  this->declare_parameter("reverse_curve_min_error", 0.12);
  this->declare_parameter("reverse_curve_threshold", 0.35);
  this->declare_parameter("reverse_curve_confirm_frames", 3);
  this->declare_parameter("reverse_curve_clear_frames", 4);
  this->declare_parameter("reverse_curve_speed", 0.16);
  this->declare_parameter("sharp_curve_threshold", 0.50);
  this->declare_parameter("sharp_curve_speed", 0.20);
  // 岔路实验专用转向预瞄；默认关闭，未设置时保留旧控制行为。
  this->declare_parameter("enable_curve_preview_steering", false);
  this->declare_parameter("curve_preview_max_far_weight", 0.45);
  this->declare_parameter("curve_preview_yaw_slew_rate", 8.0);

  // 柔性红锥避让：默认关闭，关闭时保留原固定时间 red_turning_ 控制。
  this->declare_parameter("enable_soft_red_avoidance", false);
  this->declare_parameter("red_soft_yaw", 0.30);
  this->declare_parameter("red_near_ratio", 0.35);
  // 红色目标非常靠近时，即使道路走廊判断有偏差，也强制允许避让。
  this->declare_parameter("red_very_close_ratio", 0.72);
  this->declare_parameter("red_corridor_half_width_ratio", 0.42);
  this->declare_parameter("red_soft_speed", 0.15);
  this->declare_parameter("red_soft_curve_yaw_limit", 0.15);

  // 红色目标分类与连续弯防碰撞参数。
  // 横向红线：宽高比很大且横跨画面较宽时，不作为红锥进入 soft avoidance。
  this->declare_parameter("red_line_aspect_ratio_min", 3.0);
  this->declare_parameter("red_line_min_width_ratio", 0.30);

  // 小红锥按“单个有效轮廓面积 / ROI 面积”判断，不能再把所有红色噪点面积相加。
  this->declare_parameter("red_cone_area_ratio_threshold", 0.0006);
  this->declare_parameter("red_cone_min_bbox_width", 4);
  this->declare_parameter("red_cone_min_bbox_height", 4);

  // 动态斥力：risk=0 时为 red_soft_yaw，risk=1 时放大到
  // red_soft_yaw * (1 + red_repulsion_gain)。
  this->declare_parameter("red_repulsion_gain", 1.5);

  // 当巡线正在朝红锥一侧转时，最多削弱到该比例，不能把弯道转向完全清零。
  this->declare_parameter("red_lane_keep_min_factor", 0.40);

  // 白道边界护栏：当车体已偏向避让侧（快压对侧黑线）时，平滑衰减红障斥力并恢复巡线回中力。
  this->declare_parameter("red_edge_guard_start_error", 0.15);
  this->declare_parameter("red_edge_guard_full_error", 0.30);

  // 红色避障只允许减速，不允许停车。无论红线还是红锥，最低保持该前进速度。
  this->declare_parameter("red_min_forward_speed", 0.06);

  // 红色停车线也需要绕行。若左右可用空间近似相同，使用该固定避让方向：
  // -1 = 向右绕，+1 = 向左绕。
  this->declare_parameter("red_line_fallback_direction", -1);
  this->declare_parameter("red_line_soft_yaw", 0.35);
  this->declare_parameter("red_line_speed", 0.12);

  // 实验功能（默认关闭）：红色禁行区（空心框/竖条纹）聚类合并 + 绕行方向锁存。
  this->declare_parameter("enable_red_zone_lock", false);
  this->declare_parameter("red_zone_close_kernel_w", 21);
  this->declare_parameter("red_zone_close_kernel_h", 21);
  this->declare_parameter("red_zone_min_width_ratio", 0.16);
  this->declare_parameter("red_zone_aspect_ratio_min", 0.5);
  this->declare_parameter("red_zone_aspect_ratio_max", 4.5);
  this->declare_parameter("red_zone_lock_clear_frames", 6);

  // 丢线不停车：超过 line_lost_stop_time 只切换 LOST 状态，仍低速前进。
  this->declare_parameter("line_lost_stop_time", 0.8);
  this->declare_parameter("line_lost_forward_speed", 0.06);
  this->declare_parameter("line_lost_yaw_hold_factor", 0.85);
  this->declare_parameter("speed_gain_damping", 0.5);
  this->declare_parameter("publish_debug_image", true);

  // 实验功能（默认关闭）：使用 YOLO 数字检测框屏蔽红/黄过滤中的彩色数字区域。
  this->declare_parameter("enable_yolo_digit_masking", false);
  this->declare_parameter("digit_mask_apply_to_yellow", false);
  this->declare_parameter("digit_detection_topic", "/hobot_dnn_detection");
  this->declare_parameter("digit_mask_timeout", 0.10);
  this->declare_parameter("digit_mask_min_conf", 0.40);
  this->declare_parameter("digit_mask_pad_px", 8);

  image_topic_ = this->get_parameter("image_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  debug_image_topic_ = this->get_parameter("debug_image_topic").as_string();
  line_state_topic_ = this->get_parameter("line_state_topic").as_string();
  start_topic_ = this->get_parameter("start_topic").as_string();
  stop_cmd_topic_ = this->get_parameter("stop_cmd_topic").as_string();
  line_centroid_topic_ = this->get_parameter("line_centroid_topic").as_string();

  target_speed_ = this->get_parameter("target_speed").as_double();
  max_speed_ = this->get_parameter("max_speed").as_double();
  min_speed_ = this->get_parameter("min_speed").as_double();
  max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
  kp_ = this->get_parameter("kp").as_double();

  roi_top_ratio_ = this->get_parameter("roi_top_ratio").as_double();
  roi_bottom_ratio_ = this->get_parameter("roi_bottom_ratio").as_double();

  gaussian_kernel_ = this->get_parameter("gaussian_kernel").as_int();
  if (gaussian_kernel_ <= 0) {
    gaussian_kernel_ = 1;
  }
  if (gaussian_kernel_ % 2 == 0) {
    ++gaussian_kernel_;
  }

  binary_threshold_ = this->get_parameter("binary_threshold").as_double();
  morph_kernel_ =
    std::max<int>(1, static_cast<int>(this->get_parameter("morph_kernel").as_int()));
  morph_iterations_ =
    std::max<int>(1, static_cast<int>(this->get_parameter("morph_iterations").as_int()));
  min_contour_area_ = this->get_parameter("min_contour_area").as_double();

  invert_binary_ = this->get_parameter("invert_binary").as_bool();
  use_white_filter_ = this->get_parameter("use_white_filter").as_bool();
  white_h_low_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("white_h_low").as_int()), 0, 180);
  white_h_high_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("white_h_high").as_int()), 0, 180);
  if (white_h_low_ > white_h_high_) {
    std::swap(white_h_low_, white_h_high_);
  }
  white_s_max_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("white_s_max").as_int()), 0, 255);
  white_v_min_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("white_v_min").as_int()), 0, 255);

  use_yellow_filter_ = this->get_parameter("use_yellow_filter").as_bool();
  yellow_h_low_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("yellow_h_low").as_int()), 0, 180);
  yellow_h_high_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("yellow_h_high").as_int()), 0, 180);
  if (yellow_h_low_ > yellow_h_high_) {
    std::swap(yellow_h_low_, yellow_h_high_);
  }
  yellow_s_min_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("yellow_s_min").as_int()), 0, 255);
  yellow_v_min_ = std::clamp<int>(
    static_cast<int>(this->get_parameter("yellow_v_min").as_int()), 0, 255);
  yellow_area_ratio_threshold_ = this->get_parameter("yellow_area_ratio_threshold").as_double();
  yellow_stop_duration_ = this->get_parameter("yellow_stop_duration").as_double();
  yellow_permanent_count_ = std::max<int>(
    1, static_cast<int>(this->get_parameter("yellow_permanent_count").as_int()));
  yellow_clear_frames_required_ = std::max<int>(
    1, static_cast<int>(this->get_parameter("yellow_clear_frames").as_int()));
  enable_yellow_geo_filter_ = this->get_parameter("enable_yellow_geo_filter").as_bool();
  yellow_min_width_ratio_ = clamp(
    this->get_parameter("yellow_min_width_ratio").as_double(), 0.0, 1.0);
  yellow_min_aspect_ratio_ = std::max(
    0.5, this->get_parameter("yellow_min_aspect_ratio").as_double());
  enable_yellow_odom_stop_confirm_ =
    this->get_parameter("enable_yellow_odom_stop_confirm").as_bool();
  yellow_still_speed_mps_ = std::max(
    0.0, this->get_parameter("yellow_still_speed_mps").as_double());
  yellow_still_hold_sec_ = std::max(
    0.0, this->get_parameter("yellow_still_hold_sec").as_double());
  yellow_stop_max_duration_ = std::max(
    yellow_stop_duration_, this->get_parameter("yellow_stop_max_duration").as_double());
  reverse_brake_enabled_ = this->get_parameter("enable_reverse_brake").as_bool();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  reverse_speed_ = this->get_parameter("reverse_speed").as_double();
  reverse_max_duration_ = std::max(
    0.0, this->get_parameter("reverse_max_duration").as_double());
  reverse_cutoff_speed_ = std::max(
    0.0, this->get_parameter("reverse_cutoff_speed").as_double());
  reverse_abort_speed_ = std::max(
    0.0, this->get_parameter("reverse_abort_speed").as_double());
  reverse_odom_timeout_ = std::max(
    0.0, this->get_parameter("reverse_odom_timeout").as_double());
  if (reverse_speed_ > 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "reverse_speed must be negative; disabling reverse braking for this run.");
    reverse_speed_ = 0.0;
  }
  reverse_brake_enabled_ = reverse_brake_enabled_ &&
    reverse_speed_ < 0.0 && reverse_max_duration_ > 0.0;
  enable_intersection_probe_ = this->get_parameter("enable_intersection_probe").as_bool();
  intersection_band_height_ = std::max<int>(
    1, static_cast<int>(this->get_parameter("intersection_band_height").as_int()));
  intersection_dark_ratio_threshold_ =
    this->get_parameter("intersection_dark_ratio_threshold").as_double();
  intersection_turn_speed_ = this->get_parameter("intersection_turn_speed").as_double();
  intersection_turn_yaw_rate_ = this->get_parameter("intersection_turn_yaw_rate").as_double();
  intersection_turn_duration_ = this->get_parameter("intersection_turn_duration").as_double();
  intersection_cooldown_ = this->get_parameter("intersection_cooldown").as_double();
  enable_red_filter_ = this->get_parameter("enable_red_filter").as_bool();
  red_h_low_1_ = this->get_parameter("red_h_low_1").as_int();
  red_h_high_1_ = this->get_parameter("red_h_high_1").as_int();
  red_h_low_2_ = this->get_parameter("red_h_low_2").as_int();
  red_h_high_2_ = this->get_parameter("red_h_high_2").as_int();
  red_s_min_ = this->get_parameter("red_s_min").as_int();
  red_v_min_ = this->get_parameter("red_v_min").as_int();
  red_area_ratio_threshold_ = this->get_parameter("red_area_ratio_threshold").as_double();
  red_turn_yaw_ = this->get_parameter("red_turn_yaw").as_double();
  red_turn_linear_ = this->get_parameter("red_turn_linear").as_double();
  red_turn_duration_ = this->get_parameter("red_turn_duration").as_double();
  red_vote_frames_ = std::max<int>(1, this->get_parameter("red_vote_frames").as_int());
  red_divider_angle_deg_ = this->get_parameter("red_divider_angle_deg").as_double();
  red_divider_angle_rad_ = red_divider_angle_deg_ * (M_PI / 180.0);

  line_lost_stop_time_ = this->get_parameter("line_lost_stop_time").as_double();
  speed_gain_damping_ = this->get_parameter("speed_gain_damping").as_double();
  publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();

  enable_yolo_digit_masking_ = this->get_parameter("enable_yolo_digit_masking").as_bool();
  digit_mask_apply_to_yellow_ = this->get_parameter("digit_mask_apply_to_yellow").as_bool();
  digit_detection_topic_ = this->get_parameter("digit_detection_topic").as_string();
  digit_mask_timeout_ = std::max(0.0, this->get_parameter("digit_mask_timeout").as_double());
  digit_mask_min_conf_ = clamp(
    this->get_parameter("digit_mask_min_conf").as_double(), 0.0, 1.0);
  digit_mask_pad_px_ = std::max<int>(
    0, static_cast<int>(this->get_parameter("digit_mask_pad_px").as_int()));
}

void LineFollowerNode::setup_interfaces()
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(stop_cmd_topic_, 10);
  state_pub_ = this->create_publisher<std_msgs::msg::String>(line_state_topic_, 10);
  centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>(line_centroid_topic_, 10);

  image_sub_ = image_transport::create_subscription(
    this,
    image_topic_,
    std::bind(&LineFollowerNode::image_callback, this, std::placeholders::_1),
    "raw");

  start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    start_topic_, 10, std::bind(&LineFollowerNode::start_callback, this, std::placeholders::_1));

  if (reverse_brake_enabled_ || enable_yellow_odom_stop_confirm_) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&LineFollowerNode::odom_callback, this, std::placeholders::_1));
  }

  if (reverse_brake_enabled_) {
    reverse_brake_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&LineFollowerNode::reverse_brake_timer_callback, this));
    RCLCPP_WARN(
      this->get_logger(),
      "EXPERIMENTAL reverse brake enabled: speed=%.3f m/s, max=%.3f s, "
      "cutoff=%.3f m/s, abort=%.3f m/s, odom='%s'.",
      reverse_speed_, reverse_max_duration_, reverse_cutoff_speed_,
      reverse_abort_speed_, odom_topic_.c_str());
  }

  if (enable_yellow_odom_stop_confirm_) {
    RCLCPP_WARN(
      this->get_logger(),
      "EXPERIMENTAL yellow odom standstill confirm enabled: v<=%.3f m/s, hold=%.2f s, max=%.2f s",
      yellow_still_speed_mps_, yellow_still_hold_sec_, yellow_stop_max_duration_);
  }

  if (publish_debug_image_) {
    debug_pub_ = image_transport::create_publisher(this, debug_image_topic_);
    debug_enabled_ = true;
  }

  if (enable_yolo_digit_masking_) {
    digit_det_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      digit_detection_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(&LineFollowerNode::digit_detection_callback, this, std::placeholders::_1));
    RCLCPP_WARN(
      this->get_logger(),
      "EXPERIMENTAL YOLO digit masking enabled: topic='%s', timeout=%.2fs, min_conf=%.2f, pad=%dpx",
      digit_detection_topic_.c_str(), digit_mask_timeout_, digit_mask_min_conf_, digit_mask_pad_px_);
  }
}

void LineFollowerNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool new_state = msg->data;

  if (new_state == start_enabled_) {
    return;
  }

  const bool old_state = start_enabled_;

  // 只有上一圈已经触发永久停车后，再次从 false -> true 启动，
  // 才认为开始新一圈。中途普通暂停/继续不会清零黄线计数。
  if (!old_state && new_state && permanent_stop_) {
    yellow_line_detection_count_ = 0;
    yellow_line_stop_active_ = false;
    yellow_line_latched_ = false;
    yellow_clear_frames_ = 0;
    permanent_stop_ = false;
    reverse_brake_active_ = false;
    yellow_still_started_ = false;
    yellow_line_detection_time_ = this->now();

    red_left_votes_ = 0;
    red_right_votes_ = 0;
    red_turning_ = false;
    red_turn_end_time_ = this->now();
    g_red_avoid.reset_lock();

    intersection_active_ = false;
    intersection_end_time_ = this->now();
    intersection_cooldown_until_ = this->now();

    last_detection_time_ = this->now();
    last_state_ = TrackState::LOST;

    RCLCPP_INFO(
      this->get_logger(),
      "Previous lap finished. New lap reset: yellow count=0, permanent stop cleared.");
  }

  start_enabled_ = new_state;

  // 暂停后清空连续弯状态，重新启动时由当前画面重新判断，避免继承旧状态。
  if (!start_enabled_) {
    g_curve_assist = CurveAssistRuntime{};
    g_red_avoid.reset_lock();
    yellow_still_started_ = false;
    if (reverse_brake_active_) {
      reverse_brake_active_ = false;
      publish_cmd_vel(0.0, 0.0, true);
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Start signal changed: %s -> %s",
    old_state ? "true" : "false",
    start_enabled_ ? "true" : "false");
}

void LineFollowerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_linear_speed_ = msg->twist.twist.linear.x;
  odom_angular_speed_ = msg->twist.twist.angular.z;
  odom_receive_time_ = this->now();
  odom_received_ = true;
}

void LineFollowerNode::digit_detection_callback(
  const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::vector<cv::Rect> boxes;
  for (const auto & target : msg->targets) {
    for (const auto & roi : target.rois) {
      if (roi.rect.width <= 0 || roi.rect.height <= 0) {
        continue;
      }
      if (static_cast<double>(roi.confidence) < digit_mask_min_conf_) {
        continue;
      }
      boxes.emplace_back(
        static_cast<int>(roi.rect.x_offset),
        static_cast<int>(roi.rect.y_offset),
        static_cast<int>(roi.rect.width),
        static_cast<int>(roi.rect.height));
    }
  }
  if (boxes.empty()) {
    latest_digit_boxes_.clear();
    has_digit_boxes_ = false;
    return;
  }
  latest_digit_boxes_ = std::move(boxes);
  latest_digit_steady_time_ = std::chrono::steady_clock::now();
  has_digit_boxes_ = true;
}

void LineFollowerNode::apply_digit_mask_to_roi(cv::Mat & mask, int roi_top) const
{
  if (!enable_yolo_digit_masking_ || !has_digit_boxes_ || mask.empty()) {
    return;
  }
  const double age =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_digit_steady_time_).count();
  if (age < 0.0 || age > digit_mask_timeout_) {
    return;
  }
  const cv::Rect mask_bounds(0, 0, mask.cols, mask.rows);
  for (const auto & box : latest_digit_boxes_) {
    cv::Rect roi_box(
      box.x - digit_mask_pad_px_,
      (box.y - roi_top) - digit_mask_pad_px_,
      box.width + 2 * digit_mask_pad_px_,
      box.height + 2 * digit_mask_pad_px_);
    roi_box &= mask_bounds;
    if (roi_box.width > 0 && roi_box.height > 0) {
      cv::rectangle(mask, roi_box, cv::Scalar(0), cv::FILLED);
    }
  }
}

void LineFollowerNode::reverse_brake_timer_callback()
{
  if (!reverse_brake_active_) {
    return;
  }

  const auto now = this->now();
  const double elapsed = (now - reverse_brake_start_time_).seconds();
  const double odom_age = odom_received_ ? (now - odom_receive_time_).seconds() : 1e9;
  const char * stop_reason = nullptr;

  if (!start_enabled_) {
    stop_reason = "start paused";
  } else if (elapsed >= reverse_max_duration_) {
    stop_reason = "maximum pulse duration reached";
  } else if (!odom_received_ || odom_age < 0.0 || odom_age > reverse_odom_timeout_) {
    stop_reason = "odom is not fresh";
  } else if (odom_linear_speed_ <= -reverse_abort_speed_) {
    stop_reason = "reverse motion limit reached";
  } else if (odom_linear_speed_ <= reverse_cutoff_speed_) {
    stop_reason = "forward speed cutoff reached";
  }

  if (stop_reason != nullptr) {
    reverse_brake_active_ = false;
    publish_cmd_vel(0.0, 0.0, true);
    RCLCPP_INFO(
      this->get_logger(), "Reverse brake ended: %s (odom_v=%.3f m/s, age=%.3f s).",
      stop_reason, odom_linear_speed_, odom_age);
    return;
  }

  publish_cmd_vel(reverse_speed_, 0.0, true);
}

void LineFollowerNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat frame_bgr;
  try {
    frame_bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  auto result_opt = process_image(frame_bgr);

  if (result_opt && result_opt->line_detected) {
    // 发布归一化质心（0~1，基于整幅图像）
    double norm_x = 0.5;
    double norm_y = 0.5;
    if (frame_bgr.cols > 0 && frame_bgr.rows > 0) {
      norm_x = result_opt->centroid.x / static_cast<double>(frame_bgr.cols);
      norm_y = (static_cast<double>(result_opt->roi_top) + result_opt->centroid.y) /
        static_cast<double>(frame_bgr.rows);
      norm_x = clamp(norm_x, 0.0, 1.0);
      norm_y = clamp(norm_y, 0.0, 1.0);
    }
    publish_centroid(norm_x, norm_y);
  }

  if (!start_enabled_) {
    // 停车等待期间不累计连续弯判定，避免起步继承静止画面的旧状态。
    g_curve_assist = CurveAssistRuntime{};
    g_red_avoid.reset_lock();

    static rclcpp::Time last_log_time = this->now();
    rclcpp::Time current_time = this->now();
    if ((current_time - last_log_time).seconds() > 2.0) {
      RCLCPP_INFO(this->get_logger(), "Line follower is not started, waiting for start signal...");
      last_log_time = current_time;
    }
    // 未启动时不继承上一段运动的转向预瞄状态。
    curve_preview_yaw_initialized_ = false;
    // 未启动时仍保留基础调试图。
    if (result_opt && debug_enabled_) {
      publish_debug_image(msg, frame_bgr, *result_opt);
    }

    publish_cmd_vel(0.0, 0.0, true);
    return;
  }

  const auto now = this->now();

  double linear_speed = 0.0;
  double yaw_rate = 0.0;
  TrackState current_state = TrackState::LOST;
  bool publish_stop_cmd = false;

  // ============================================================
  // 红色避障检测与投票
  //
  // 这一版仍保持原来的 detect_red() 接口，但是：
  // 1) detect_red 内部区分“横向红色停车线”和“红锥”，两类都需要避障；
  // 2) 近远使用 bbox 底部位置，而不是中心点；
  // 3) soft 模式只在红色目标进入近场 + 当前行驶走廊附近时激活；
  // 4) 连续弯中的实际动态斥力在后面的控制融合区完成；
  // 5) 红色目标没有停车权限，只允许绕行和减速。
  // ============================================================
  const bool soft_red_enabled =
    this->get_parameter("enable_soft_red_avoidance").as_bool();
  const double red_soft_yaw =
    std::max(0.0, this->get_parameter("red_soft_yaw").as_double());
  const double red_near_ratio = clamp(
    this->get_parameter("red_near_ratio").as_double(), 0.0, 1.0);
  const double red_very_close_ratio = clamp(
    this->get_parameter("red_very_close_ratio").as_double(), 0.0, 1.0);
  const double red_corridor_half_width_ratio = clamp(
    this->get_parameter("red_corridor_half_width_ratio").as_double(), 0.0, 1.0);
  const double red_soft_speed =
    std::max(0.0, this->get_parameter("red_soft_speed").as_double());
  const double red_min_forward_speed =
    std::max(0.01, this->get_parameter("red_min_forward_speed").as_double());

  const double line_lost_forward_speed =
    std::max(0.01, this->get_parameter("line_lost_forward_speed").as_double());
  const double line_lost_yaw_hold_factor =
    clamp(this->get_parameter("line_lost_yaw_hold_factor").as_double(), 0.0, 1.0);

  bool soft_red_active = false;
  double soft_red_yaw_bias = 0.0;

  // 每帧先清理“控制阶段”调试状态；detect_red() 会填充当前帧目标几何信息。
  g_red_avoid.in_corridor = false;
  g_red_avoid.risk = 0.0;
  g_red_avoid.very_close = false;

  if (enable_red_filter_ && result_opt && result_opt->line_detected) {
    int roi_top = result_opt->roi_top;
    int roi_bottom = result_opt->roi_bottom;
    cv::Point2d line_centroid_roi(result_opt->centroid.x, result_opt->centroid.y);

    double side_sign = 0.0;
    cv::Point2d red_center_img, div_base_img, div_target_img;
    bool red_seen = detect_red(
      frame_bgr, roi_top, roi_bottom, line_centroid_roi, side_sign,
      red_center_img, div_base_img, div_target_img);

    last_red_overlay_valid_ = red_seen;

    if (red_seen) {
      last_red_center_img_ = red_center_img;
      last_div_base_img_ = div_base_img;
      last_div_target_img_ = div_target_img;

      // 图像坐标系 y 向下。detect_red() 当前叉积定义：
      // side_sign < 0 -> 红锥在当前行驶方向左侧；
      // side_sign > 0 -> 红锥在当前行驶方向右侧。
      if (side_sign < 0.0) {
        red_left_votes_++;
        red_right_votes_ = 0;
      } else {
        red_right_votes_++;
        red_left_votes_ = 0;
      }

      if (soft_red_enabled) {
        // 柔性模式不使用固定时间 red_turning_。
        red_turning_ = false;

        // 关键变化：使用红锥 bbox 的“底部”来估计近远。
        // 小红锥的中心可能还很高，但底部已经非常接近车头。
        const double red_bottom_ratio = clamp(
          g_red_avoid.bottom_ratio, 0.0, 1.0);

        const bool vote_confirmed =
          red_left_votes_ >= red_vote_frames_ ||
          red_right_votes_ >= red_vote_frames_;

        // 用红锥底部中心计算其到当前行驶方向线的横向距离。
        const cv::Point2d corridor_v = div_target_img - div_base_img;
        const cv::Point2d red_v = g_red_avoid.bottom_img - div_base_img;
        const double corridor_norm =
          std::hypot(corridor_v.x, corridor_v.y);

        double lateral_distance = 1e9;
        if (corridor_norm > 1e-6) {
          lateral_distance = std::abs(
            corridor_v.x * red_v.y - corridor_v.y * red_v.x) / corridor_norm;
        }

        const double corridor_limit =
          red_corridor_half_width_ratio * static_cast<double>(frame_bgr.cols);
        const bool red_in_corridor =
          lateral_distance <= corridor_limit;

        g_red_avoid.in_corridor = red_in_corridor;

        // 两级触发：
        // 1) 正常近场：必须在当前行驶走廊附近；
        // 2) 非常近：允许走廊判断有一定误差，避免急弯中漏掉贴脸红锥。
        const bool red_close =
          red_bottom_ratio >= red_near_ratio;
        const bool red_very_close =
          red_bottom_ratio >= red_very_close_ratio;

        // 仅当红障尚未完全错开到画面最外侧（内侧边缘仍侵入中间 44% 区域）时，
        // 才允许 red_very_close 绕过走廊检查，避免已错过的贴边红锥继续把车推出对侧黑线。
        const double frame_w = std::max(1.0, static_cast<double>(frame_bgr.cols));
        const double red_left_ratio = static_cast<double>(g_red_avoid.bbox_img.x) / frame_w;
        const double red_right_ratio =
          static_cast<double>(g_red_avoid.bbox_img.x + g_red_avoid.bbox_img.width) / frame_w;
        const bool red_still_blocking =
          (side_sign < 0.0) ? (red_right_ratio >= 0.28) : (red_left_ratio <= 0.72);

        if (vote_confirmed &&
            red_close &&
            red_still_blocking &&
            (red_in_corridor || red_very_close))
        {
          soft_red_active = true;

          // 这里只记录方向符号。真正的动态斥力大小在后面根据 risk 计算。
          // 红线与红锥使用不同的基础避障角速度，但都不允许停车。
          const double base_avoid_yaw =
            g_red_avoid.is_red_line ?
            std::max(0.0, this->get_parameter("red_line_soft_yaw").as_double()) :
            red_soft_yaw;

          soft_red_yaw_bias =
            (side_sign < 0.0) ? -base_avoid_yaw : base_avoid_yaw;
        }
      } else {
        // 旧模式：保持原固定时间控制，便于旧参数文件继续使用。
        if (red_left_votes_ >= red_vote_frames_) {
          const int dir = -1;
          red_turn_yaw_cmd_ =
            clamp(dir * red_turn_yaw_, -max_yaw_rate_, max_yaw_rate_);
          red_turning_ = true;
          red_turn_end_time_ =
            now + rclcpp::Duration::from_seconds(red_turn_duration_);
          red_left_votes_ = 0;
          red_right_votes_ = 0;
        } else if (red_right_votes_ >= red_vote_frames_) {
          const int dir = 1;
          red_turn_yaw_cmd_ =
            clamp(dir * red_turn_yaw_, -max_yaw_rate_, max_yaw_rate_);
          red_turning_ = true;
          red_turn_end_time_ =
            now + rclcpp::Duration::from_seconds(red_turn_duration_);
          red_left_votes_ = 0;
          red_right_votes_ = 0;
        }
      }
    } else {
      red_left_votes_ = 0;
      red_right_votes_ = 0;

      g_red_avoid.decay_lock(
        this->get_parameter("enable_red_zone_lock").as_bool(),
        static_cast<int>(this->get_parameter("red_zone_lock_clear_frames").as_int()));

      if (soft_red_enabled) {
        red_turning_ = false;
      } else if (now >= red_turn_end_time_) {
        red_turning_ = false;
      }
    }
  } else {
    last_red_overlay_valid_ = false;
    g_red_avoid.reset_frame();

    g_red_avoid.decay_lock(
      enable_red_filter_ && this->get_parameter("enable_red_zone_lock").as_bool(),
      static_cast<int>(this->get_parameter("red_zone_lock_clear_frames").as_int()));

    if (soft_red_enabled) {
      red_left_votes_ = 0;
      red_right_votes_ = 0;
      red_turning_ = false;
    }
  }

  // ============================================================
  // 黄色线计数：一条实体黄线只允许计数一次
  //
  // 第一次看到黄线：锁存 + count++ + 停车
  // 黄线持续存在：不重复计数、不重复触发
  // 连续 yellow_clear_frames_required_ 帧看不到黄色：解除锁存
  // ============================================================
  const bool yellow_detected =
    result_opt && result_opt->yellow_line_detected;

  if (yellow_detected) {
    yellow_clear_frames_ = 0;

    if (!yellow_line_latched_) {
      yellow_line_latched_ = true;
      yellow_line_detection_count_++;
      yellow_line_detection_time_ = now;

      reverse_brake_active_ = false;
      if (reverse_brake_enabled_) {
        const double odom_age = odom_received_ ? (now - odom_receive_time_).seconds() : 1e9;
        const bool odom_fresh = odom_received_ && odom_age >= 0.0 &&
          odom_age <= reverse_odom_timeout_;
        if (odom_fresh && odom_linear_speed_ > reverse_cutoff_speed_ &&
          odom_linear_speed_ > -reverse_abort_speed_)
        {
          reverse_brake_active_ = true;
          reverse_brake_start_time_ = now;
          // 立即清除上一条前进指令；20 ms 定时回调再发有界反向脉冲。
          publish_cmd_vel(0.0, 0.0, true);
          RCLCPP_WARN(
            this->get_logger(),
            "Yellow reverse brake triggered: yellow_count=%d, yellow_area_threshold=%.5f, "
            "odom_v=%.3f m/s.",
            yellow_line_detection_count_, yellow_area_ratio_threshold_, odom_linear_speed_);
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Reverse brake skipped: odom is stale or speed is already at/below cutoff "
            "(odom_v=%.3f m/s, age=%.3f s). Zero-speed yellow stop remains active.",
            odom_linear_speed_, odom_age);
        }
      }

      RCLCPP_INFO(
        this->get_logger(),
        "New yellow line detected! Count: %d/%d",
        yellow_line_detection_count_,
        yellow_permanent_count_);

      if (yellow_line_detection_count_ >= yellow_permanent_count_) {
        permanent_stop_ = true;
        yellow_line_stop_active_ = false;

        RCLCPP_INFO(
          this->get_logger(),
          "Final yellow line detected. Permanent stop activated.");
      } else {
        yellow_line_stop_active_ = true;
        yellow_still_started_ = false;

        RCLCPP_INFO(
          this->get_logger(),
          "Temporary yellow stop activated for %.2f s.",
          yellow_stop_duration_);
      }
    }
  } else if (yellow_line_latched_) {
    yellow_clear_frames_++;

    if (yellow_clear_frames_ >= yellow_clear_frames_required_) {
      yellow_line_latched_ = false;
      yellow_clear_frames_ = 0;

      RCLCPP_INFO(
        this->get_logger(),
        "Yellow line cleared. Ready for next yellow line.");
    }
  }

  const double since_yellow_detection =
    (now - yellow_line_detection_time_).seconds();

  bool temp_yellow_stop_active = false;
  if (!permanent_stop_ && yellow_line_stop_active_) {
    if (!enable_yellow_odom_stop_confirm_) {
      if (since_yellow_detection < yellow_stop_duration_) {
        temp_yellow_stop_active = true;
      } else {
        yellow_line_stop_active_ = false;
        RCLCPP_INFO(
          this->get_logger(),
          "Yellow line stop duration ended. Resuming line following.");
      }
    } else {
      const double odom_age = odom_received_ ? (now - odom_receive_time_).seconds() : 1e9;
      const double odom_timeout = std::max(reverse_odom_timeout_, 0.30);
      const bool odom_fresh =
        odom_received_ && odom_age >= 0.0 && odom_age <= odom_timeout;
      const bool timed_out = since_yellow_detection >= yellow_stop_max_duration_;
      if (!odom_fresh) {
        yellow_still_started_ = false;
        if (timed_out) {
          yellow_line_stop_active_ = false;
          RCLCPP_WARN(
            this->get_logger(),
            "Yellow line stop ended (max_timeout while odom stale): elapsed=%.2fs odom_age=%.2fs.",
            since_yellow_detection, odom_age);
        } else {
          temp_yellow_stop_active = true;
        }
      } else {
        const bool speed_still =
          std::abs(odom_linear_speed_) <= yellow_still_speed_mps_ &&
          std::abs(odom_angular_speed_) <= 0.05;
        if (speed_still) {
          if (!yellow_still_started_) {
            yellow_still_started_ = true;
            yellow_still_start_time_ = now;
          }
        } else {
          yellow_still_started_ = false;
        }
        const double still_elapsed =
          yellow_still_started_ ? (now - yellow_still_start_time_).seconds() : 0.0;
        const bool hold_done =
          (since_yellow_detection >= yellow_stop_duration_) &&
          (still_elapsed >= yellow_still_hold_sec_);
        if (hold_done || timed_out) {
          yellow_line_stop_active_ = false;
          yellow_still_started_ = false;
          RCLCPP_INFO(
            this->get_logger(),
            "Yellow line stop ended (%s): elapsed=%.2fs still=%.2fs odom_v=%.3f odom_w=%.3f.",
            timed_out ? "max_timeout" : "standstill_confirmed",
            since_yellow_detection, still_elapsed, odom_linear_speed_, odom_angular_speed_);
        } else {
          temp_yellow_stop_active = true;
        }
      }
    }
  }

  // ============================================================
  // 交叉口探针
  // ============================================================
  if (enable_intersection_probe_ && result_opt && result_opt->line_detected &&
    !intersection_active_ && now >= intersection_cooldown_until_ &&
    !yellow_line_stop_active_ && !permanent_stop_)
  {
    const cv::Mat & mask = result_opt->binary_mask;
    if (!mask.empty()) {
      int band_h = std::min(intersection_band_height_, mask.rows);
      if (band_h > 0) {
        cv::Rect band_rect(0, 0, mask.cols, band_h);
        cv::Mat band = mask(band_rect);
        int nonzero = cv::countNonZero(band);
        double total = static_cast<double>(band.total());
        double black_ratio =
          total > 0.0 ? 1.0 - static_cast<double>(nonzero) / total : 0.0;

        if (black_ratio >= intersection_dark_ratio_threshold_) {
          intersection_active_ = true;
          intersection_end_time_ =
            now + rclcpp::Duration::from_seconds(intersection_turn_duration_);
          intersection_cooldown_until_ =
            now + rclcpp::Duration::from_seconds(intersection_cooldown_);

          RCLCPP_INFO(
            this->get_logger(),
            "Intersection probe triggered: black_ratio=%.2f band_h=%d",
            black_ratio, band_h);
        }
      }
    }
  }

  // ============================================================
  // 基础控制：黄色停车 > 普通巡线 > 丢线低速继续
  // ============================================================
  if (permanent_stop_) {
    current_state = TrackState::YELLOW_LINE_DETECTED;
    linear_speed = 0.0;
    yaw_rate = 0.0;
    publish_stop_cmd = true;
  } else if (temp_yellow_stop_active) {
    current_state = TrackState::YELLOW_LINE_DETECTED;
    linear_speed = 0.0;
    yaw_rate = 0.0;
    publish_stop_cmd = true;
  } else {
    if (result_opt && result_opt->line_detected) {
      const auto & result = *result_opt;
      last_detection_time_ = now;
      current_state = TrackState::TRACKING;

      const double yaw_cmd =
        clamp(-(kp_ * result.error), -max_yaw_rate_, max_yaw_rate_);

      const double composite_err = std::abs(result.error);
      const double speed_scale =
        1.0 - speed_gain_damping_ * std::min(1.0, composite_err);
      double speed_cmd =
        clamp(target_speed_ * speed_scale, min_speed_, max_speed_);

      // 连续弯辅助只改变“特殊路段”的速度上限。
      // 普通弯仍保持原来的 70/30 目标点和 P 控制。
      if (this->get_parameter("enable_continuous_curve_assist").as_bool()) {
        const double reverse_curve_speed =
          std::max(0.0, this->get_parameter("reverse_curve_speed").as_double());
        const double sharp_curve_threshold =
          std::max(0.0, this->get_parameter("sharp_curve_threshold").as_double());
        const double sharp_curve_speed =
          std::max(0.0, this->get_parameter("sharp_curve_speed").as_double());

        if (g_curve_assist.reverse_curve_active) {
          speed_cmd = std::min(speed_cmd, reverse_curve_speed);
        } else if (g_curve_assist.curve_level >= sharp_curve_threshold) {
          speed_cmd = std::min(speed_cmd, sharp_curve_speed);
        }
      }

      linear_speed = speed_cmd;
      yaw_rate = yaw_cmd;
    } else {
      const double since_detection =
        (now - last_detection_time_).seconds();

      // 丢线也不能停车。
      // 短时间沿用上一帧转向并低速前进；超过阈值只把状态标成 LOST，
      // 但仍保持最低前进速度，不发布 /cmd_vel_stop。
      current_state =
        since_detection >= line_lost_stop_time_ ?
        TrackState::LOST : last_state_;

      linear_speed = line_lost_forward_speed;
      yaw_rate =
        g_motion.valid ?
        clamp(
          g_motion.last_yaw * line_lost_yaw_hold_factor,
          -max_yaw_rate_,
          max_yaw_rate_) :
        0.0;

      publish_stop_cmd = false;
    }
  }

  // ============================================================
  // 交叉口/红色控制只有在黄色停车未激活时才能覆盖巡线
  // ============================================================
  const bool yellow_stop_now =
    permanent_stop_ || temp_yellow_stop_active;

  if (!yellow_stop_now && intersection_active_) {
    if (now <= intersection_end_time_) {
      linear_speed = intersection_turn_speed_;
      yaw_rate = clamp(intersection_turn_yaw_rate_, -max_yaw_rate_, max_yaw_rate_);
      current_state = TrackState::TRACKING;
    } else {
      intersection_active_ = false;
    }
  }

  if (!yellow_stop_now && soft_red_enabled) {
    if (soft_red_active) {
      // ==========================================================
      // 动态红锥避障融合
      //
      // 核心原则：
      // - 红锥越近，斥力越强、速度越低；
      // - 如果巡线正在朝红锥方向转，逐步削弱这部分巡线力；
      // - 但不能把弯道转向完全清零，避免为了躲锥直接冲出白路；
      // - 连续弯远处仍限制红锥修正，越近越平滑解除限制；
      // - 极近时继续增强避让并降到最低前进速度，但绝不停车。
      // ==========================================================

      const double bottom_ratio =
        clamp(g_red_avoid.bottom_ratio, 0.0, 1.0);

      const double risk_denominator =
        std::max(1e-3, red_very_close_ratio - red_near_ratio);

      const double risk =
        clamp(
          (bottom_ratio - red_near_ratio) / risk_denominator,
          0.0,
          1.0);

      g_red_avoid.risk = risk;

      const double repulsion_gain =
        std::max(
          0.0,
          this->get_parameter("red_repulsion_gain").as_double());

      const double lane_keep_min_factor =
        clamp(
          this->get_parameter("red_lane_keep_min_factor").as_double(),
          0.0,
          1.0);

      // 红色目标最低前进速度已在 image_callback 顶部读取。

      // soft_red_yaw_bias 只用来判断红锥在左还是右。
      // < 0：红锥在左，需要向右避（angular.z 变负）
      // > 0：红锥在右，需要向左避（angular.z 变正）
      const bool red_on_left =
        soft_red_yaw_bias < 0.0;

      // 斥力只保留“大小”，方向单独处理，避免符号重复导致反向撞锥。
      double repulsion_strength =
        std::abs(soft_red_yaw_bias) *
        (1.0 + repulsion_gain * risk);

      // 连续反向弯：
      // risk=0 时仍保持原来的 curve_bias_limit，
      // risk 越高，允许的红锥修正逐渐增大到 max_yaw_rate_。
      // 不再使用“到某个阈值突然解除”的跳变方式。
      if (g_curve_assist.reverse_curve_active) {
        const double curve_bias_limit =
          clamp(
            this->get_parameter("red_soft_curve_yaw_limit").as_double(),
            0.0,
            max_yaw_rate_);

        const double dynamic_limit =
          curve_bias_limit +
          risk * (max_yaw_rate_ - curve_bias_limit);

        repulsion_strength =
          std::min(repulsion_strength, dynamic_limit);
      }

      // 当巡线控制正在把车朝红锥方向带时，随风险增大逐步削弱。
      // risk=1 时仍至少保留 lane_keep_min_factor，防止连续弯完全失去道路约束。
      const double lane_keep_factor =
        1.0 -
        (1.0 - lane_keep_min_factor) * risk;

      const double edge_guard_start = clamp(
        this->get_parameter("red_edge_guard_start_error").as_double(), 0.0, 0.9);
      const double edge_guard_full = clamp(
        this->get_parameter("red_edge_guard_full_error").as_double(),
        edge_guard_start + 0.02, 1.0);
      const double line_err =
        (result_opt && result_opt->line_detected) ? result_opt->error : 0.0;
      // result.error > 0 表示白道在右（车偏左靠近左黑线）；< 0 表示白道在左（车偏右靠近右黑线）
      const double edge_err =
        red_on_left ? std::max(0.0, -line_err) : std::max(0.0, line_err);
      const double edge_guard = clamp(
        (edge_err - edge_guard_start) / (edge_guard_full - edge_guard_start),
        0.0, 1.0);

      repulsion_strength *= (1.0 - edge_guard);
      const double effective_lane_keep_factor =
        lane_keep_factor + (1.0 - lane_keep_factor) * edge_guard;

      if (red_on_left) {
        // 红锥在左 -> 应向右避（angular.z 为负）。
        if (yaw_rate > 0.0) {
          // 当前巡线正在向左（朝红锥）转，削弱巡线力并施加向右斥力。
          yaw_rate = yaw_rate * effective_lane_keep_factor - repulsion_strength;
        } else {
          // 巡线本身已在向右转（例如右弯外侧红锥）：同向不叠加，避免双重右转切内弯压内圈黑线。
          yaw_rate = std::min(yaw_rate, -repulsion_strength);
        }
      } else {
        // 红锥在右 -> 应向左避（angular.z 为正）。
        if (yaw_rate < 0.0) {
          // 当前巡线正在向右（朝红锥）转，削弱巡线力并施加向左斥力。
          yaw_rate = yaw_rate * effective_lane_keep_factor + repulsion_strength;
        } else {
          // 巡线本身已在向左转（例如左弯外侧红锥）：同向不叠加，避免双重左转切内弯压内圈黑线。
          yaw_rate = std::max(yaw_rate, repulsion_strength);
        }
      }

      yaw_rate =
        clamp(
          yaw_rate,
          -max_yaw_rate_,
          max_yaw_rate_);

      // 越近越慢：
      // risk=0 -> red_soft_speed
      // risk=1 -> 至少 red_min_forward_speed
      const double red_base_speed =
        g_red_avoid.is_red_line ?
        std::max(0.0, this->get_parameter("red_line_speed").as_double()) :
        red_soft_speed;

      const double safe_speed =
        std::max(
          red_min_forward_speed,
          red_base_speed * (1.0 - 0.60 * risk));

      // 红色目标只能减速，不能停车。
      // 即使非常靠近，也至少保持 red_min_forward_speed 向前运动。
      linear_speed =
        std::max(
          red_min_forward_speed,
          std::min(linear_speed, safe_speed));

      g_red_avoid.very_close =
        bottom_ratio >= red_very_close_ratio;

      current_state = TrackState::TRACKING;
    }
  } else if (!yellow_stop_now && red_turning_ && now < red_turn_end_time_) {
    // 旧红锥控制：仅在 enable_soft_red_avoidance=false 时生效。
    linear_speed = std::max(red_min_forward_speed, red_turn_linear_);
    yaw_rate = red_turn_yaw_cmd_;
    current_state = TrackState::TRACKING;
  } else if (red_turning_ && now >= red_turn_end_time_) {
    red_turning_ = false;
  }

  // ============================================================
  // 最终安全锁：黄色停车拥有最高优先级，任何后续控制都不能覆盖
  // ============================================================
  if (permanent_stop_ || temp_yellow_stop_active) {
    current_state = TrackState::YELLOW_LINE_DETECTED;
    linear_speed = 0.0;
    yaw_rate = 0.0;
    publish_stop_cmd = true;
  }

  // 只有黄色停车锁可以把运行中的车速压到 0。
  // 记录最近一次正常运动命令，供短暂丢线时继续低速前进。
  if (!yellow_stop_now && linear_speed > 0.0) {
    g_motion.valid = true;
    g_motion.last_linear = linear_speed;
    g_motion.last_yaw = yaw_rate;
  }

  // 红色检测和风险计算已经完成后再发布调试图，
  // 避免上一版调试红点总是落后一帧。
  if (result_opt && debug_enabled_) {
    publish_debug_image(msg, frame_bgr, *result_opt);
  }

  // 岔路实验模式限制正常巡线转向指令的变化率，避免目标点切换造成机械跳变。
  // 停车、丢线、未启用预瞄及反向脉冲均绕过该限制。
  const bool curve_preview_enabled =
    this->get_parameter("enable_curve_preview_steering").as_bool();
  if (curve_preview_enabled && current_state == TrackState::TRACKING &&
      result_opt && result_opt->line_detected && !publish_stop_cmd &&
      !reverse_brake_active_ && !red_turning_)
  {
    const double slew_rate = std::max(
      0.1, this->get_parameter("curve_preview_yaw_slew_rate").as_double());
    if (!curve_preview_yaw_initialized_) {
      curve_preview_last_yaw_rate_ = yaw_rate;
      curve_preview_yaw_initialized_ = true;
    } else {
      const double dt = clamp((now - curve_preview_last_time_).seconds(), 0.0, 0.10);
      const double max_delta = slew_rate * dt;
      yaw_rate = clamp(
        yaw_rate,
        curve_preview_last_yaw_rate_ - max_delta,
        curve_preview_last_yaw_rate_ + max_delta);
      curve_preview_last_yaw_rate_ = yaw_rate;
    }
    curve_preview_last_time_ = now;
  } else {
    curve_preview_yaw_initialized_ = false;
  }

  // 反向脉冲期间由 20 ms 定时器独占输出；图像回调不得用普通停车零速覆盖它。
  if (!reverse_brake_active_) {
    publish_cmd_vel(linear_speed, yaw_rate, publish_stop_cmd);
  }

  if (current_state != last_state_ ||
      current_state == TrackState::LOST ||
      current_state == TrackState::YELLOW_LINE_DETECTED)
  {
    publish_state(current_state);
  }

  last_state_ = current_state;
}

std::optional<ProcessingResult> LineFollowerNode::process_image(
  const cv::Mat & frame_bgr) const
{
  if (frame_bgr.empty()) {
    return std::nullopt;
  }

  const int height = frame_bgr.rows;
  const int width = frame_bgr.cols;

  // ============================================================
  // 1. ROI
  // ============================================================
  const int roi_top = std::clamp(
    static_cast<int>(height * (1.0 - roi_top_ratio_)),
    0,
    height - 1);

  const int roi_bottom = std::clamp(
    height - static_cast<int>(height * roi_bottom_ratio_),
    roi_top + 1,
    height);

  const int roi_height = roi_bottom - roi_top;

  const cv::Rect roi_rect(0, roi_top, width, roi_height);
  const cv::Mat roi = frame_bgr(roi_rect);

  // ============================================================
  // 2. 高斯滤波 + HSV
  // ============================================================
  cv::Mat blurred;
  cv::GaussianBlur(
    roi,
    blurred,
    cv::Size(gaussian_kernel_, gaussian_kernel_),
    0);

  cv::Mat hsv;
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

  // ============================================================
  // 3. 黄色线独立检测
  // ============================================================
  bool yellow_line_detected = false;

  if (use_yellow_filter_) {
    cv::Mat yellow_mask;

    const cv::Scalar lower_yellow(
      static_cast<double>(yellow_h_low_),
      static_cast<double>(yellow_s_min_),
      static_cast<double>(yellow_v_min_));

    const cv::Scalar upper_yellow(
      static_cast<double>(yellow_h_high_),
      255.0,
      255.0);

    cv::inRange(hsv, lower_yellow, upper_yellow, yellow_mask);
    if (digit_mask_apply_to_yellow_) {
      apply_digit_mask_to_roi(yellow_mask, roi_top);
    }

    const int yellow_pixels = cv::countNonZero(yellow_mask);
    const double yellow_area_ratio =
      yellow_mask.total() > 0 ?
      static_cast<double>(yellow_pixels) /
        static_cast<double>(yellow_mask.total()) :
      0.0;

    yellow_line_detected =
      yellow_area_ratio >= yellow_area_ratio_threshold_;

    if (yellow_line_detected && enable_yellow_geo_filter_) {
      std::vector<std::vector<cv::Point>> y_contours;
      cv::findContours(
        yellow_mask, y_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      bool geo_ok = false;
      const double min_w = static_cast<double>(width) * yellow_min_width_ratio_;
      for (const auto & yc : y_contours) {
        const cv::Rect ybox = cv::boundingRect(yc);
        if (ybox.width <= 0 || ybox.height <= 0) {
          continue;
        }
        const double y_aspect =
          static_cast<double>(ybox.width) / static_cast<double>(ybox.height);
        if (static_cast<double>(ybox.width) >= min_w &&
            y_aspect >= yellow_min_aspect_ratio_)
        {
          geo_ok = true;
          break;
        }
      }
      yellow_line_detected = geo_ok;
    }
  }

  // ============================================================
  // 4. 白色道路 mask
  //
  // use_white_filter=true 时只使用 HSV 白色 mask，
  // 不再和灰度 threshold 做 OR，减少环境高亮/反光污染。
  // ============================================================
  cv::Mat binary;

  if (use_white_filter_) {
    const cv::Scalar lower_white(
      static_cast<double>(white_h_low_),
      0.0,
      static_cast<double>(white_v_min_));

    const cv::Scalar upper_white(
      static_cast<double>(white_h_high_),
      static_cast<double>(white_s_max_),
      255.0);

    cv::inRange(hsv, lower_white, upper_white, binary);
  } else {
    cv::Mat gray;
    cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);

    const int threshold_type =
      invert_binary_ ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

    cv::threshold(
      gray,
      binary,
      binary_threshold_,
      255,
      threshold_type);
  }

  // ============================================================
  // 5. 形态学：先去小噪声，再补小断裂
  // ============================================================
  cv::morphologyEx(
    binary,
    binary,
    cv::MORPH_OPEN,
    morph_element_,
    cv::Point(-1, -1),
    1);

  cv::morphologyEx(
    binary,
    binary,
    cv::MORPH_CLOSE,
    morph_element_,
    cv::Point(-1, -1),
    morph_iterations_);

  ProcessingResult result{};
  result.line_detected = false;
  result.yellow_line_detected = yellow_line_detected;
  result.roi_bgr = roi.clone();
  result.binary_mask = binary.clone();
  result.roi_top = roi_rect.y;
  result.roi_bottom = roi_rect.y + roi_rect.height;

  // ============================================================
  // 6. 最大轮廓只做存在性判断 / fallback / debug
  // ============================================================
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    binary,
    contours,
    cv::RETR_EXTERNAL,
    cv::CHAIN_APPROX_SIMPLE);

  bool contour_valid = false;
  cv::Point2f contour_centroid(
    static_cast<float>(width) * 0.5F,
    static_cast<float>(roi_height) * 0.70F);

  if (!contours.empty()) {
    double max_area = 0.0;
    size_t best_index = 0;

    for (size_t i = 0; i < contours.size(); ++i) {
      const double area = cv::contourArea(contours[i]);
      if (area > max_area) {
        max_area = area;
        best_index = i;
      }
    }

    if (max_area >= min_contour_area_) {
      contour_valid = true;
      result.bounding_box = cv::boundingRect(contours[best_index]);

      const auto moments = cv::moments(contours[best_index]);
      if (moments.m00 != 0.0) {
        contour_centroid.x =
          static_cast<float>(moments.m10 / moments.m00);
        contour_centroid.y =
          static_cast<float>(moments.m01 / moments.m00);
      } else {
        contour_centroid.x =
          static_cast<float>(
            result.bounding_box.x + result.bounding_box.width / 2.0);
        contour_centroid.y =
          static_cast<float>(
            result.bounding_box.y + result.bounding_box.height / 2.0);
      }
    }
  }

  // ============================================================
  // 7. 局部前视白色通道中心
  //
  // 不再用“整块白色区域面积质心”直接控制。
  // 在局部横向带中找连续白色通道，并选择最接近参考位置的通道。
  // ============================================================
  auto find_band_center =
    [&](int y0, int y1, double reference_x) -> std::optional<double>
    {
      y0 = std::clamp(y0, 0, binary.rows - 1);
      y1 = std::clamp(y1, y0 + 1, binary.rows);

      const cv::Mat band =
        binary(cv::Rect(0, y0, binary.cols, y1 - y0));

      const int band_height = band.rows;
      if (band_height <= 0) {
        return std::nullopt;
      }

      // 一列中至少 30% 像素是白色，才认为属于道路。
      constexpr double kColumnWhiteRatio = 0.30;
      const int min_white_pixels = std::max(
        1,
        static_cast<int>(
          static_cast<double>(band_height) * kColumnWhiteRatio));

      std::vector<unsigned char> active(binary.cols, 0);

      for (int x = 0; x < binary.cols; ++x) {
        if (cv::countNonZero(band.col(x)) >= min_white_pixels) {
          active[x] = 1;
        }
      }

      // 太窄的连续白色区域视为反光/噪声。
      constexpr int kMinRunWidth = 20;

      bool found = false;
      double best_center = reference_x;
      double best_distance = 1e9;
      int best_width = 0;
      int run_start = -1;

      for (int x = 0; x <= binary.cols; ++x) {
        const bool is_active =
          (x < binary.cols) ? (active[x] != 0) : false;

        if (is_active && run_start < 0) {
          run_start = x;
        }

        if ((!is_active || x == binary.cols) && run_start >= 0) {
          const int run_end = x - 1;
          const int run_width = run_end - run_start + 1;

          if (run_width >= kMinRunWidth) {
            const double center =
              0.5 * static_cast<double>(run_start + run_end);

            double distance;
            if (reference_x >= static_cast<double>(run_start) &&
                reference_x <= static_cast<double>(run_end))
            {
              distance = 0.0;
            } else {
              distance = std::abs(center - reference_x);
            }

            if (!found ||
                distance < best_distance ||
                (std::abs(distance - best_distance) < 1e-6 &&
                 run_width > best_width))
            {
              found = true;
              best_center = center;
              best_distance = distance;
              best_width = run_width;
            }
          }

          run_start = -1;
        }
      }

      if (!found) {
        return std::nullopt;
      }

      return best_center;
    };

  // ============================================================
  // 8. 双采样带
  // near：当前车前道路，权重大
  // far ：提前感知弯道方向
  // ============================================================
  const int far_y0 = static_cast<int>(static_cast<double>(roi_height) * 0.34);
  const int far_y1 = static_cast<int>(static_cast<double>(roi_height) * 0.56);
  const int near_y0 = static_cast<int>(static_cast<double>(roi_height) * 0.62);
  const int near_y1 = static_cast<int>(static_cast<double>(roi_height) * 0.88);

  const double image_center_x = static_cast<double>(width) * 0.5;

  const auto near_center =
    find_band_center(near_y0, near_y1, image_center_x);

  const double far_reference =
    near_center.has_value() ? near_center.value() : image_center_x;

  const auto far_center =
    find_band_center(far_y0, far_y1, far_reference);

  // ============================================================
  // 9. 连续反向弯判别
  //
  // 普通左/右弯：near 与 far 同方向，继续原来的 70/30。
  // 连续反向弯：near 与 far 分居中心线两侧，且两者偏差都足够大、
  //             差值也足够大，并连续确认若干帧后才进入。
  // 进入后先跟 near，避免 near/far 相互抵消；far 主要用于提前降速。
  // ============================================================
  const bool curve_assist_enabled =
    this->get_parameter("enable_continuous_curve_assist").as_bool();
  const double reverse_curve_min_error =
    std::max(0.0, this->get_parameter("reverse_curve_min_error").as_double());
  const double reverse_curve_threshold =
    std::max(0.0, this->get_parameter("reverse_curve_threshold").as_double());
  const int reverse_curve_confirm_frames = std::max(
    1, static_cast<int>(
      this->get_parameter("reverse_curve_confirm_frames").as_int()));
  const int reverse_curve_clear_frames = std::max(
    1, static_cast<int>(
      this->get_parameter("reverse_curve_clear_frames").as_int()));

  const bool near_valid = near_center.has_value();
  const bool far_valid = far_center.has_value();

  double near_error = 0.0;
  double far_error = 0.0;

  if (near_valid) {
    near_error =
      (near_center.value() - image_center_x) / image_center_x;
    near_error = clamp(near_error, -1.0, 1.0);
  }
  if (far_valid) {
    far_error =
      (far_center.value() - image_center_x) / image_center_x;
    far_error = clamp(far_error, -1.0, 1.0);
  }

  g_curve_assist.near_valid = near_valid;
  g_curve_assist.far_valid = far_valid;
  g_curve_assist.near_error = near_error;
  g_curve_assist.far_error = far_error;
  g_curve_assist.curve_level = 0.0;

  if (near_valid) {
    g_curve_assist.curve_level =
      std::max(g_curve_assist.curve_level, std::abs(near_error));
  }
  if (far_valid) {
    g_curve_assist.curve_level =
      std::max(g_curve_assist.curve_level, std::abs(far_error));
  }

  bool reverse_curve_candidate = false;
  if (curve_assist_enabled && near_valid && far_valid) {
    const bool opposite_direction = near_error * far_error < 0.0;
    const bool both_significant =
      std::abs(near_error) >= reverse_curve_min_error &&
      std::abs(far_error) >= reverse_curve_min_error;
    const bool large_difference =
      std::abs(far_error - near_error) >= reverse_curve_threshold;

    reverse_curve_candidate =
      opposite_direction && both_significant && large_difference;
  }

  if (!curve_assist_enabled) {
    g_curve_assist.reverse_curve_active = false;
    g_curve_assist.reverse_confirm_count = 0;
    g_curve_assist.reverse_clear_count = 0;
  } else if (reverse_curve_candidate) {
    g_curve_assist.reverse_clear_count = 0;
    g_curve_assist.reverse_confirm_count++;

    if (!g_curve_assist.reverse_curve_active &&
      g_curve_assist.reverse_confirm_count >= reverse_curve_confirm_frames)
    {
      g_curve_assist.reverse_curve_active = true;
      g_curve_assist.reverse_confirm_count = reverse_curve_confirm_frames;
    }
  } else {
    g_curve_assist.reverse_confirm_count = 0;

    if (g_curve_assist.reverse_curve_active) {
      g_curve_assist.reverse_clear_count++;
      if (g_curve_assist.reverse_clear_count >= reverse_curve_clear_frames) {
        g_curve_assist.reverse_curve_active = false;
        g_curve_assist.reverse_clear_count = 0;
      }
    } else {
      g_curve_assist.reverse_clear_count = 0;
    }
  }

  // ============================================================
  // 10. 生成最终目标点
  // ============================================================
  double target_x = static_cast<double>(contour_centroid.x);
  double target_y = static_cast<double>(contour_centroid.y);
  bool local_target_valid = false;

  if (near_valid && far_valid) {
    const bool curve_preview_enabled =
      this->get_parameter("enable_curve_preview_steering").as_bool();
    if (curve_preview_enabled) {
      // On an S bend, the near and far targets can point in opposite
      // directions. Use the far target to anticipate the next turn while the
      // near target is still close to center, then hand control back to near
      // as the current turn becomes visible. Once the existing S-curve detector
      // confirms the transition, follow near only to avoid cancelling the
      // current turn with the next one.
      const double max_far_weight = clamp(
        this->get_parameter("curve_preview_max_far_weight").as_double(), 0.30, 0.65);
      double far_weight = 0.30;
      if (near_error * far_error < 0.0) {
        if (curve_assist_enabled && g_curve_assist.reverse_curve_active) {
          far_weight = 0.0;
        } else {
          const double near_turn_progress = clamp(
            std::abs(near_error) / 0.25, 0.0, 1.0);
          const double preview_weight =
            max_far_weight * (1.0 - near_turn_progress);
          far_weight = std::max(0.0, preview_weight);

          if (curve_assist_enabled && reverse_curve_candidate) {
            const int confirm_frames = std::max(
              1, reverse_curve_confirm_frames);
            const double confirm_progress = clamp(
              static_cast<double>(g_curve_assist.reverse_confirm_count) /
              static_cast<double>(confirm_frames), 0.0, 1.0);
            far_weight *= (1.0 - confirm_progress);
          }
        }
      } else {
        const double far_lead = std::max(
          0.0, std::abs(far_error) - std::abs(near_error));
        const double anticipation = clamp(far_lead / 0.20, 0.0, 1.0);
        far_weight += (max_far_weight - 0.30) * anticipation;
      }

      const double near_y = 0.5 * static_cast<double>(near_y0 + near_y1);
      const double far_y = 0.5 * static_cast<double>(far_y0 + far_y1);
      target_x = (1.0 - far_weight) * near_center.value() +
        far_weight * far_center.value();
      target_y = (1.0 - far_weight) * near_y + far_weight * far_y;
    } else if (curve_assist_enabled && g_curve_assist.reverse_curve_active) {
      // Default path is unchanged: follow near during a confirmed S transition.
      target_x = near_center.value();
      target_y = 0.5 * static_cast<double>(near_y0 + near_y1);
    } else {
      // Default path remains the original 70% near + 30% far.
      target_x = 0.70 * near_center.value() + 0.30 * far_center.value();

      const double near_y =
        0.5 * static_cast<double>(near_y0 + near_y1);
      const double far_y =
        0.5 * static_cast<double>(far_y0 + far_y1);

      target_y = 0.70 * near_y + 0.30 * far_y;
    }
    local_target_valid = true;
  } else if (near_valid) {
    target_x = near_center.value();
    target_y = 0.5 * static_cast<double>(near_y0 + near_y1);
    local_target_valid = true;
  } else if (far_valid) {
    target_x = far_center.value();
    target_y = 0.5 * static_cast<double>(far_y0 + far_y1);
    local_target_valid = true;
  }

  // 优先使用局部前视结果；只有局部完全失败时才退回原最大轮廓质心。
  if (!local_target_valid && !contour_valid) {
    return result;
  }

  result.line_detected = true;
  result.centroid = cv::Point2f(
    static_cast<float>(target_x),
    static_cast<float>(target_y));

  result.error =
    (target_x - image_center_x) / image_center_x;
  result.error = clamp(result.error, -1.0, 1.0);
  result.state = TrackState::TRACKING;

  return result;
}

bool LineFollowerNode::detect_red(
  const cv::Mat & frame_bgr, int roi_top, int roi_bottom,
  const cv::Point2d & line_centroid_roi, double & side_sign,
  cv::Point2d & red_centroid_img, cv::Point2d & div_base_img,
  cv::Point2d & div_target_img) const
{
  side_sign = 0.0;

  // 每次检测先清空当前帧几何结果（保留 dir_locked 等跨帧锁存状态）。
  g_red_avoid.reset_frame();

  if (!enable_red_filter_ || frame_bgr.empty()) {
    return false;
  }

  roi_top = std::clamp(roi_top, 0, frame_bgr.rows - 1);
  roi_bottom = std::clamp(roi_bottom, roi_top + 1, frame_bgr.rows);

  const cv::Rect roi_rect(
    0,
    roi_top,
    frame_bgr.cols,
    roi_bottom - roi_top);

  const cv::Mat roi =
    frame_bgr(roi_rect);

  cv::Mat hsv;
  cv::cvtColor(
    roi,
    hsv,
    cv::COLOR_BGR2HSV);

  cv::Mat mask1;
  cv::Mat mask2;
  cv::Mat mask;

  cv::inRange(
    hsv,
    cv::Scalar(
      static_cast<double>(red_h_low_1_),
      static_cast<double>(red_s_min_),
      static_cast<double>(red_v_min_)),
    cv::Scalar(
      static_cast<double>(red_h_high_1_),
      255.0,
      255.0),
    mask1);

  cv::inRange(
    hsv,
    cv::Scalar(
      static_cast<double>(red_h_low_2_),
      static_cast<double>(red_s_min_),
      static_cast<double>(red_v_min_)),
    cv::Scalar(
      static_cast<double>(red_h_high_2_),
      255.0,
      255.0),
    mask2);

  cv::bitwise_or(
    mask1,
    mask2,
    mask);
  apply_digit_mask_to_roi(mask, roi_top);

  // 红锥在远处时面积很小。
  // 先做轻量开运算去散点，再闭运算补小孔，避免把一个红锥切成很多碎块。
  const cv::Mat red_kernel =
    cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(3, 3));

  cv::morphologyEx(
    mask,
    mask,
    cv::MORPH_OPEN,
    red_kernel,
    cv::Point(-1, -1),
    1);

  cv::morphologyEx(
    mask,
    mask,
    cv::MORPH_CLOSE,
    red_kernel,
    cv::Point(-1, -1),
    1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    mask,
    contours,
    cv::RETR_EXTERNAL,
    cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    return false;
  }

  const double line_aspect_min =
    std::max(
      1.0,
      this->get_parameter("red_line_aspect_ratio_min").as_double());

  const double line_width_ratio_min =
    clamp(
      this->get_parameter("red_line_min_width_ratio").as_double(),
      0.0,
      1.0);

  const double cone_area_ratio_min =
    std::max(
      0.0,
      this->get_parameter("red_cone_area_ratio_threshold").as_double());

  const int cone_min_bbox_width =
    std::max(
      1,
      static_cast<int>(
        this->get_parameter("red_cone_min_bbox_width").as_int()));

  const int cone_min_bbox_height =
    std::max(
      1,
      static_cast<int>(
        this->get_parameter("red_cone_min_bbox_height").as_int()));

  int best_cone_index = -1;
  double best_cone_area = 0.0;

  int best_line_index = -1;
  double best_line_area = 0.0;

  const double roi_area =
    std::max(
      1.0,
      static_cast<double>(mask.total()));

  const bool red_zone_lock_enabled =
    this->get_parameter("enable_red_zone_lock").as_bool();

  bool zone_found = false;
  cv::Rect best_zone_box{};
  double best_zone_area = 0.0;

  if (red_zone_lock_enabled) {
    const int zone_kw = std::max(
      1, static_cast<int>(this->get_parameter("red_zone_close_kernel_w").as_int()));
    const int zone_kh = std::max(
      1, static_cast<int>(this->get_parameter("red_zone_close_kernel_h").as_int()));
    const double zone_min_w_ratio = clamp(
      this->get_parameter("red_zone_min_width_ratio").as_double(), 0.0, 1.0);
    const double zone_aspect_min = std::max(
      0.1, this->get_parameter("red_zone_aspect_ratio_min").as_double());
    const double zone_aspect_max = std::max(
      zone_aspect_min, this->get_parameter("red_zone_aspect_ratio_max").as_double());

    cv::Mat zone_mask;
    const cv::Mat zone_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(zone_kw, zone_kh));
    cv::morphologyEx(
      mask, zone_mask, cv::MORPH_CLOSE, zone_kernel, cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> zone_contours;
    cv::findContours(
      zone_mask, zone_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> raw_boxes;
    raw_boxes.reserve(contours.size());
    for (const auto & rc : contours) {
      if (cv::contourArea(rc) > 0.0) {
        const cv::Rect rb = cv::boundingRect(rc);
        if (rb.width > 0 && rb.height > 0) {
          raw_boxes.push_back(rb);
        }
      }
    }

    for (const auto & zc : zone_contours) {
      const double z_area = cv::contourArea(zc);
      if (z_area <= 0.0 || (z_area / roi_area) < cone_area_ratio_min) {
        continue;
      }
      const cv::Rect z_box = cv::boundingRect(zc);
      if (z_box.width <= 0 || z_box.height <= 0) {
        continue;
      }
      const double z_aspect =
        static_cast<double>(z_box.width) / static_cast<double>(z_box.height);
      if (z_aspect < zone_aspect_min || z_aspect > zone_aspect_max ||
          static_cast<double>(z_box.width) <
            static_cast<double>(frame_bgr.cols) * zone_min_w_ratio)
      {
        continue;
      }

      int inner_fragments = 0;
      for (const auto & rb : raw_boxes) {
        if ((rb & z_box).area() > 0) {
          inner_fragments++;
        }
      }
      const bool wide_horizontal_tape =
        z_aspect >= 2.2 &&
        static_cast<double>(z_box.width) >= static_cast<double>(frame_bgr.cols) * 0.22;
      if (inner_fragments >= 2 || wide_horizontal_tape) {
        if (z_area > best_zone_area) {
          best_zone_area = z_area;
          best_zone_box = z_box;
          zone_found = true;
        }
      }
    }
  }

  // 关键变化：
  // 不再“先找最大红色，再判断是不是红线”。
  // 而是逐个轮廓先做红线过滤，再从剩余候选里寻找最大的红锥。
  for (size_t i = 0; i < contours.size(); ++i) {
    const double area =
      cv::contourArea(contours[i]);

    if (area <= 0.0) {
      continue;
    }

    const cv::Rect box =
      cv::boundingRect(contours[i]);

    if (box.width <= 0 ||
        box.height <= 0)
    {
      continue;
    }

    const double aspect_ratio =
      static_cast<double>(box.width) /
      static_cast<double>(box.height);

    const bool looks_like_red_line =
      aspect_ratio >= line_aspect_min &&
      static_cast<double>(box.width) >=
        static_cast<double>(frame_bgr.cols) * line_width_ratio_min;

    if (looks_like_red_line) {
      if (area > best_line_area) {
        best_line_area = area;
        best_line_index = static_cast<int>(i);
      }
      continue;
    }

    // 太小的 bbox 先当噪声。
    if (box.width < cone_min_bbox_width ||
        box.height < cone_min_bbox_height)
    {
      continue;
    }

    // 面积判断改为“当前这个轮廓本身 / ROI”，
    // 不允许多个分散红噪点靠总面积把一个小噪声抬成红锥。
    const double contour_area_ratio =
      area / roi_area;

    if (contour_area_ratio < cone_area_ratio_min) {
      continue;
    }

    if (area > best_cone_area) {
      best_cone_area = area;
      best_cone_index = static_cast<int>(i);
    }
  }

  // 记录横向红色停车线/禁行区，供调试显示。
  if (zone_found) {
    g_red_avoid.red_line_detected = true;
    g_red_avoid.red_line_bbox_img =
      cv::Rect(
        best_zone_box.x,
        best_zone_box.y + roi_top,
        best_zone_box.width,
        best_zone_box.height);
  } else if (best_line_index >= 0) {
    const cv::Rect line_box_roi =
      cv::boundingRect(
        contours[static_cast<size_t>(best_line_index)]);

    g_red_avoid.red_line_detected = true;
    g_red_avoid.red_line_bbox_img =
      cv::Rect(
        line_box_roi.x,
        line_box_roi.y + roi_top,
        line_box_roi.width,
        line_box_roi.height);
  }

  // 合并禁行区优先于零散碎片；否则优先处理红锥，其次普通红线。
  const bool use_red_line =
    zone_found || (best_cone_index < 0 && best_line_index >= 0);

  if (!zone_found && best_cone_index < 0 && best_line_index < 0) {
    return false;
  }

  const cv::Rect box = zone_found ? best_zone_box :
    cv::boundingRect(
      contours[static_cast<size_t>(use_red_line ? best_line_index : best_cone_index)]);

  const double selected_area = zone_found ? best_zone_area :
    (use_red_line ? best_line_area : best_cone_area);

  const double aspect_ratio =
    static_cast<double>(box.width) /
    static_cast<double>(
      std::max(1, box.height));

  const double contour_area_ratio =
    selected_area / roi_area;

  const cv::Point2d red_centroid_roi(
    static_cast<double>(box.x) +
      static_cast<double>(box.width) * 0.5,
    static_cast<double>(box.y) +
      static_cast<double>(box.height) * 0.5);

  // 红锥“底部中心”比 bbox 中心更适合估计与车的近远。
  const cv::Point2d red_bottom_roi(
    static_cast<double>(box.x) +
      static_cast<double>(box.width) * 0.5,
    static_cast<double>(box.y + box.height));

  // 以 ROI 底部中心为根点，连向巡线目标，判断红锥在当前行驶方向左/右。
  cv::Point2d base(
    static_cast<double>(mask.cols) * 0.5,
    static_cast<double>(mask.rows));

  cv::Point2d v_line =
    line_centroid_roi - base;

  if (red_divider_angle_rad_ != 0.0) {
    const double angle =
      v_line.x >= 0.0 ?
      red_divider_angle_rad_ :
      -red_divider_angle_rad_;

    const double c =
      std::cos(angle);

    const double s =
      std::sin(angle);

    v_line =
      cv::Point2d(
        v_line.x * c - v_line.y * s,
        v_line.x * s + v_line.y * c);
  }

  const cv::Point2d v_red =
    red_centroid_roi - base;

  if (use_red_line) {
    // 横向停车线通常横跨道路，单纯用中心点叉积会在中心附近左右抖动。
    // 优先选择 bbox 两侧剩余空间更大的一边绕行；两边近似相等时使用固定方向。
    const double left_space =
      static_cast<double>(box.x);
    const double right_space =
      static_cast<double>(mask.cols - (box.x + box.width));

    if (right_space > left_space + 10.0) {
      // 右侧空间更大 -> 向右绕；按现有语义等价于“障碍在左”。
      side_sign = -1.0;
    } else if (left_space > right_space + 10.0) {
      // 左侧空间更大 -> 向左绕。
      side_sign = 1.0;
    } else {
      const int fallback_direction =
        static_cast<int>(
          this->get_parameter("red_line_fallback_direction").as_int());

      // fallback_direction: -1=向右，+1=向左。
      side_sign = fallback_direction >= 0 ? 1.0 : -1.0;
    }
  } else {
    // 图像坐标系 y 轴向下：
    // cross < 0 -> 红锥在当前行驶方向左侧；
    // cross >=0 -> 红锥在当前行驶方向右侧。
    const double cross =
      v_line.x * v_red.y -
      v_line.y * v_red.x;

    side_sign =
      cross >= 0.0 ?
      1.0 :
      -1.0;
  }

  const double roi_h =
    std::max(
      1,
      roi_bottom - roi_top);

  const double computed_bottom_ratio =
    clamp(
      red_bottom_roi.y / static_cast<double>(roi_h),
      0.0,
      1.0);

  if (red_zone_lock_enabled) {
    const double lock_near_ratio = clamp(
      this->get_parameter("red_near_ratio").as_double(), 0.0, 1.0);
    if (g_red_avoid.dir_locked) {
      side_sign = g_red_avoid.locked_side_sign;
      g_red_avoid.dir_clear_frames = 0;
    } else if (computed_bottom_ratio >= lock_near_ratio) {
      g_red_avoid.dir_locked = true;
      g_red_avoid.locked_side_sign = side_sign;
      g_red_avoid.dir_clear_frames = 0;
    }
  } else {
    g_red_avoid.reset_lock();
  }

  // 转成整图坐标。
  red_centroid_img =
    cv::Point2d(
      red_centroid_roi.x,
      red_centroid_roi.y +
        static_cast<double>(roi_top));

  const cv::Point2d red_bottom_img(
    red_bottom_roi.x,
    red_bottom_roi.y +
      static_cast<double>(roi_top));

  div_base_img =
    cv::Point2d(
      base.x,
      base.y +
        static_cast<double>(roi_top));

  const cv::Point2d rotated_target =
    base + v_line;

  div_target_img =
    cv::Point2d(
      rotated_target.x,
      rotated_target.y +
        static_cast<double>(roi_top));

  // 保存当前红色目标几何信息，供 image_callback 动态避障和调试图使用。
  g_red_avoid.valid = true;
  g_red_avoid.is_red_line = use_red_line;
  g_red_avoid.bbox_img =
    cv::Rect(
      box.x,
      box.y + roi_top,
      box.width,
      box.height);

  g_red_avoid.center_img =
    red_centroid_img;

  g_red_avoid.bottom_img =
    red_bottom_img;

  g_red_avoid.aspect_ratio =
    aspect_ratio;

  g_red_avoid.contour_area_ratio =
    contour_area_ratio;

  g_red_avoid.bottom_ratio = computed_bottom_ratio;

  return true;
}

std::string LineFollowerNode::state_to_string(TrackState state) const
{
  switch (state) {
    case TrackState::TRACKING:
      return "tracking";
    case TrackState::YELLOW_LINE_DETECTED:
      return "yellow_line_detected";
    case TrackState::LOST:
    default:
      return "lost";
  }
}

void LineFollowerNode::publish_cmd_vel(double linear_speed, double yaw_rate, bool publish_stop){
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed;
  cmd.angular.z = yaw_rate;
  cmd_vel_pub_->publish(cmd);

  // Yellow stop topic has highest mux priority. Reverse braking also uses it so
  // the temporary pulse cannot be overwritten by the normal follower command.
  if (stop_pub_ && publish_stop)
  {
    stop_pub_->publish(cmd);
  }
}

void LineFollowerNode::publish_state(TrackState state)
{
  std_msgs::msg::String msg;
  msg.data = state_to_string(state);
  state_pub_->publish(msg);
}

void LineFollowerNode::publish_centroid(double norm_x, double norm_y)
{
  if (!centroid_pub_) {
    return;
  }
  geometry_msgs::msg::Point msg;
  msg.x = norm_x;
  msg.y = norm_y;
  centroid_pub_->publish(msg);
}

void LineFollowerNode::publish_debug_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & source,
  const cv::Mat & frame_bgr,
  const ProcessingResult & result) const
{
  if (!debug_enabled_ || result.binary_mask.empty()) {
    return;
  }

  cv::Mat overlay = frame_bgr.clone();
  cv::Mat mask_color;
  cv::cvtColor(result.binary_mask, mask_color, cv::COLOR_GRAY2BGR);

  cv::Mat blended;
  cv::addWeighted(result.roi_bgr, 0.6, mask_color, 0.4, 0.0, blended);

  blended.copyTo(
    overlay(cv::Rect(
      0,
      result.roi_top,
      result.roi_bgr.cols,
      result.roi_bgr.rows)));

  // 只有白色巡线有效时才绘制道路框、目标红点和分界线。
  if (result.line_detected) {
    if (result.bounding_box.width > 0 && result.bounding_box.height > 0) {
      cv::rectangle(
        overlay,
        cv::Rect(
          result.bounding_box.x,
          result.bounding_box.y + result.roi_top,
          result.bounding_box.width,
          result.bounding_box.height),
        cv::Scalar(0, 255, 0),
        2);
    }

    const cv::Point centroid(
      static_cast<int>(result.centroid.x),
      static_cast<int>(result.centroid.y) + result.roi_top);

    // 红点 = 新的“局部前视白色通道中心”目标点
    cv::circle(overlay, centroid, 8, cv::Scalar(0, 0, 255), -1);

    cv::line(
      overlay,
      cv::Point(overlay.cols / 2, result.roi_top),
      cv::Point(overlay.cols / 2, result.roi_bottom),
      cv::Scalar(0, 255, 255),
      2);

    // 底部中心 -> 当前巡线目标点
    cv::Point2d base(
      static_cast<double>(frame_bgr.cols) * 0.5,
      static_cast<double>(result.roi_bottom));

    cv::Point2d v_line(
      static_cast<double>(result.centroid.x) - base.x,
      static_cast<double>(result.centroid.y + result.roi_top) - base.y);

    if (red_divider_angle_rad_ != 0.0) {
      const double angle =
        v_line.x >= 0.0 ? red_divider_angle_rad_ : -red_divider_angle_rad_;
      const double c = std::cos(angle);
      const double s = std::sin(angle);
      v_line = cv::Point2d(
        v_line.x * c - v_line.y * s,
        v_line.x * s + v_line.y * c);
    }

    const cv::Point2d target = base + v_line;
    cv::line(overlay, base, target, cv::Scalar(0, 255, 255), 2);

    // 当前帧红锥调试信息。
    if (last_red_overlay_valid_ && g_red_avoid.valid) {
      cv::rectangle(
        overlay,
        g_red_avoid.bbox_img,
        cv::Scalar(0, 0, 255),
        2);

      cv::circle(
        overlay,
        g_red_avoid.center_img,
        6,
        cv::Scalar(0, 0, 255),
        -1);

      // 红锥底部中心：用黄色点表示实际用于近远判断的位置。
      cv::circle(
        overlay,
        g_red_avoid.bottom_img,
        6,
        cv::Scalar(0, 255, 255),
        -1);

      char red_text[200];
      std::snprintf(
        red_text,
        sizeof(red_text),
        "%s bottom=%.2f risk=%.2f area=%.4f %s%s",
        g_red_avoid.is_red_line ? "RED_LINE" : "CONE",
        g_red_avoid.bottom_ratio,
        g_red_avoid.risk,
        g_red_avoid.contour_area_ratio,
        g_red_avoid.in_corridor ? "IN_PATH " : "OUT_PATH ",
        g_red_avoid.very_close ? "VERY_CLOSE" : "");

      cv::putText(
        overlay,
        red_text,
        cv::Point(10, 90),
        cv::FONT_HERSHEY_SIMPLEX,
        0.50,
        cv::Scalar(0, 0, 255),
        2);
    }

    // 横向红色停车线单独标记；它同样会进入避障，但不会停车。
    if (g_red_avoid.red_line_detected) {
      cv::rectangle(
        overlay,
        g_red_avoid.red_line_bbox_img,
        cv::Scalar(255, 0, 255),
        2);

      cv::putText(
        overlay,
        "RED LINE AVOID",
        cv::Point(
          g_red_avoid.red_line_bbox_img.x,
          std::max(
            20,
            g_red_avoid.red_line_bbox_img.y - 5)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.50,
        cv::Scalar(255, 0, 255),
        2);
    }
  }

  if (this->get_parameter("enable_continuous_curve_assist").as_bool()) {
    const char * curve_mode =
      g_curve_assist.reverse_curve_active ? "REV_CURVE" : "NORMAL";
    char curve_text[160];
    std::snprintf(
      curve_text, sizeof(curve_text),
      "%s near=%+.2f far=%+.2f level=%.2f",
      curve_mode, g_curve_assist.near_error, g_curve_assist.far_error,
      g_curve_assist.curve_level);
    cv::putText(
      overlay, curve_text, cv::Point(10, 60),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);
  }

  if (result.yellow_line_detected) {
    cv::putText(
      overlay,
      "YELLOW LINE DETECTED!",
      cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX,
      1.0,
      cv::Scalar(0, 255, 255),
      2);
  }

  cv_bridge::CvImage debug_image;
  debug_image.header = source->header;
  debug_image.encoding = "bgr8";
  debug_image.image = overlay;

  debug_pub_.publish(debug_image.toImageMsg());
}

double LineFollowerNode::clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace line_follower

