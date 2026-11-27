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

  // 横向红色停车线 / 镂空红色禁行区 / 红锥统一进入避障，按最近威胁（bottom_ratio）排序。
  bool red_line_detected{false};
  bool is_red_line{false};
  bool is_red_zone{false};
  cv::Rect red_line_bbox_img{};

  // 当前选中的最近威胁红色目标。
  cv::Rect bbox_img{};
  cv::Point2d center_img{};
  cv::Point2d bottom_img{};

  double aspect_ratio{0.0};
  double contour_area_ratio{0.0};
  double bottom_ratio{0.0};
  double effective_near_ratio{0.38};

  bool in_corridor{false};
  bool still_blocking{false};
  double risk{0.0};
  bool very_close{false};

  double gap_left_l{0.0};
  double gap_left_r{0.0};
  double w_left_px{0.0};
  double gap_right_l{0.0};
  double gap_right_r{0.0};
  double w_right_px{0.0};
  bool edge_clipped{false};
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
  red_committed_dir_ = 0;
  red_clear_frames_ = 0;
  red_turning_ = false;
  red_turn_end_time_ = this->now();
  last_red_overlay_valid_ = false;
  prev_err_valid_ = false;
  prev_error_ = 0.0;
  filtered_d_err_ = 0.0;
  prev_err_time_ = this->now();
  odom_total_dist_m_ = 0.0;
  tjunction_armed_ = false;
  tjunction_active_ = false;
  tjunction_done_ = false;
  tjunction_arm_dist_m_ = 0.0;
  tjunction_last_det_time_ = this->now();
  tjunction_start_time_ = this->now();
  tjunction_narrow_frames_ = 0;
  tjunction_wall_confirm_ = 0;

  RCLCPP_INFO(
    this->get_logger(),
    "Line follower subscribing to '%s', publishing cmd_vel on '%s', start_topic: '%s'",
    image_topic_.c_str(), cmd_vel_topic_.c_str(), start_topic_.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    "System features: kp=%.2f, kd=%.3f, avoid_topic='%s', det_cmd='%s', "
    "red_zone_cluster=%s, tjunction_merge=%s",
    kp_, kd_, avoid_cmd_topic_.c_str(), detection_cmd_topic_.c_str(),
    this->get_parameter("enable_red_zone_cluster").as_bool() ? "on" : "off",
    enable_tjunction_merge_ ? "on" : "off");

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
  this->declare_parameter("avoid_cmd_topic", "/cmd_vel_avoid");
  this->declare_parameter("detection_cmd_topic", "/cmd_vel_detection");
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
  this->declare_parameter("kd", 0.0);

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
  this->declare_parameter("base_far_weight", 0.30);
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
  this->declare_parameter("red_edge_guard_start_error", 0.25);
  this->declare_parameter("red_edge_guard_full_error", 0.50);
  this->declare_parameter("red_edge_guard_min_repulsion", 0.25);

  // 镂空红色禁行区（24x13cm 细红条纹）聚类检测与方向锁存参数
  this->declare_parameter("enable_red_zone_cluster", true);
  this->declare_parameter("red_zone_min_width_ratio", 0.10);
  this->declare_parameter("red_zone_min_pixels", 120);
  this->declare_parameter("red_zone_near_ratio", 0.26);
  this->declare_parameter("red_zone_soft_yaw", 0.24);
  this->declare_parameter("red_latch_clear_frames", 5);

  // 岔路内支路 ┤ 丁字路口安全并线（默认关闭，仅在岔路图配置开启）
  this->declare_parameter("enable_tjunction_merge", false);
  this->declare_parameter("tjunction_merge_direction", -1);
  this->declare_parameter("tjunction_arm_yellow_count", 2);
  this->declare_parameter("tjunction_min_arm_dist", 0.55);
  this->declare_parameter("tjunction_min_arm_time", 2.0);
  this->declare_parameter("tjunction_min_narrow_frames", 8);
  this->declare_parameter("tjunction_min_full_rows", 18);
  this->declare_parameter("tjunction_merge_speed", 0.13);
  this->declare_parameter("tjunction_merge_yaw", 1.35);
  this->declare_parameter("tjunction_min_duration", 0.85);
  this->declare_parameter("tjunction_max_duration", 2.35);

  // 红色避障只允许减速，不允许停车。无论红线还是红锥，最低保持该前进速度。
  this->declare_parameter("red_min_forward_speed", 0.06);

  // 红色停车线也需要绕行。若左右可用空间近似相同，使用该固定避让方向：
  // -1 = 向右绕，+1 = 向左绕。
  this->declare_parameter("red_line_fallback_direction", -1);
  this->declare_parameter("red_line_soft_yaw", 0.35);
  this->declare_parameter("red_line_speed", 0.12);

  // 丢线不停车：超过 line_lost_stop_time 只切换 LOST 状态，仍低速前进。
  this->declare_parameter("line_lost_stop_time", 0.8);
  this->declare_parameter("line_lost_forward_speed", 0.06);
  this->declare_parameter("line_lost_yaw_hold_factor", 0.85);
  this->declare_parameter("speed_gain_damping", 0.5);
  this->declare_parameter("publish_debug_image", true);

  image_topic_ = this->get_parameter("image_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  avoid_cmd_topic_ = this->get_parameter("avoid_cmd_topic").as_string();
  detection_cmd_topic_ = this->get_parameter("detection_cmd_topic").as_string();
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
  kd_ = std::max(0.0, this->get_parameter("kd").as_double());

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
  red_latch_clear_frames_ = std::max<int>(1, static_cast<int>(this->get_parameter("red_latch_clear_frames").as_int()));
  red_divider_angle_deg_ = this->get_parameter("red_divider_angle_deg").as_double();
  red_divider_angle_rad_ = red_divider_angle_deg_ * (M_PI / 180.0);

  enable_tjunction_merge_ = this->get_parameter("enable_tjunction_merge").as_bool();
  tjunction_merge_direction_ =
    this->get_parameter("tjunction_merge_direction").as_int() >= 0 ? 1 : -1;
  tjunction_arm_yellow_count_ = std::max<int>(
    0, static_cast<int>(this->get_parameter("tjunction_arm_yellow_count").as_int()));
  tjunction_min_arm_dist_ = std::max(0.0, this->get_parameter("tjunction_min_arm_dist").as_double());
  tjunction_min_arm_time_ = std::max(0.0, this->get_parameter("tjunction_min_arm_time").as_double());
  tjunction_min_narrow_frames_ = std::max<int>(
    1, static_cast<int>(this->get_parameter("tjunction_min_narrow_frames").as_int()));
  tjunction_min_full_rows_ = std::max<int>(
    5, static_cast<int>(this->get_parameter("tjunction_min_full_rows").as_int()));
  tjunction_merge_speed_ = std::max(0.05, this->get_parameter("tjunction_merge_speed").as_double());
  tjunction_merge_yaw_ = std::max(0.3, this->get_parameter("tjunction_merge_yaw").as_double());
  tjunction_min_duration_ = std::max(0.2, this->get_parameter("tjunction_min_duration").as_double());
  tjunction_max_duration_ = std::max(
    tjunction_min_duration_ + 0.2, this->get_parameter("tjunction_max_duration").as_double());

  line_lost_stop_time_ = this->get_parameter("line_lost_stop_time").as_double();
  speed_gain_damping_ = this->get_parameter("speed_gain_damping").as_double();
  publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
}

void LineFollowerNode::setup_interfaces()
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  if (!avoid_cmd_topic_.empty() && avoid_cmd_topic_ != cmd_vel_topic_) {
    avoid_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(avoid_cmd_topic_, 10);
  }
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

  // Always subscribe to odom (used by reverse brake and T-junction post-fork travel gate).
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10),
    std::bind(&LineFollowerNode::odom_callback, this, std::placeholders::_1));

  if (enable_tjunction_merge_ && !detection_cmd_topic_.empty()) {
    det_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      detection_cmd_topic_, rclcpp::QoS(10),
      std::bind(&LineFollowerNode::detection_cmd_callback, this, std::placeholders::_1));
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

  if (publish_debug_image_) {
    debug_pub_ = image_transport::create_publisher(this, debug_image_topic_);
    debug_enabled_ = true;
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
    yellow_line_detection_time_ = this->now();

    red_left_votes_ = 0;
    red_right_votes_ = 0;
    red_committed_dir_ = 0;
    red_clear_frames_ = 0;
    red_turning_ = false;
    red_turn_end_time_ = this->now();
    prev_err_valid_ = false;
    filtered_d_err_ = 0.0;
    tjunction_armed_ = false;
    tjunction_active_ = false;
    tjunction_done_ = false;
    tjunction_narrow_frames_ = 0;
    tjunction_wall_confirm_ = 0;

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

  // 暂停后清空连续弯、避障锁存与微分历史，重新启动时由当前画面重新判断。
  if (!start_enabled_) {
    g_curve_assist = CurveAssistRuntime{};
    red_committed_dir_ = 0;
    red_clear_frames_ = 0;
    prev_err_valid_ = false;
    filtered_d_err_ = 0.0;
    tjunction_active_ = false;
    tjunction_wall_confirm_ = 0;
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
  const auto now = this->now();
  const double v = msg->twist.twist.linear.x;
  if (odom_received_) {
    const double dt = (now - odom_receive_time_).seconds();
    if (dt > 0.0 && dt < 0.5) {
      odom_total_dist_m_ += std::abs(v) * dt;
    }
  }
  odom_linear_speed_ = v;
  odom_receive_time_ = now;
  odom_received_ = true;
}

void LineFollowerNode::detection_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!enable_tjunction_merge_ || !start_enabled_ || !msg) {
    return;
  }
  const auto now = this->now();
  // On fork map, turning right (angular.z < -0.25) after yellow count == tjunction_arm_yellow_count_
  // enters the inner shortcut branch that terminates at the ┤ T-junction.
  if (yellow_line_detection_count_ == tjunction_arm_yellow_count_ &&
      msg->angular.z <= -0.25 && !tjunction_done_)
  {
    if (!tjunction_armed_) {
      tjunction_armed_ = true;
      tjunction_narrow_frames_ = 0;
      tjunction_wall_confirm_ = 0;
      RCLCPP_INFO(
        this->get_logger(),
        "[T-merge] Inner shortcut branch entered (det ang=%.2f at yellow=%d). "
        "Armed (waiting >=%.2fm / %.1fs & %d narrow frames).",
        msg->angular.z, yellow_line_detection_count_,
        tjunction_min_arm_dist_, tjunction_min_arm_time_, tjunction_min_narrow_frames_);
    }
    // Keep updating anchor until the digit turn finishes so post-turn travel distance
    // is measured strictly AFTER exiting the fork-2 Y-split.
    tjunction_last_det_time_ = now;
    tjunction_arm_dist_m_ = odom_total_dist_m_;
    tjunction_narrow_frames_ = 0;
    tjunction_wall_confirm_ = 0;
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

  if (!start_enabled_) {
    // 停车等待期间不累计连续弯判定、不发布有效质心，避免起步前/终点触发数字转向。
    g_curve_assist = CurveAssistRuntime{};

    static rclcpp::Time last_log_time = this->now();
    rclcpp::Time current_time = this->now();
    if ((current_time - last_log_time).seconds() > 2.0) {
      RCLCPP_INFO(this->get_logger(), "Line follower is not started, waiting for start signal...");
      last_log_time = current_time;
    }
    // 未启动时不继承上一段运动的转向预瞄状态。
    curve_preview_yaw_initialized_ = false;
    prev_err_valid_ = false;
    // 未启动时仍保留基础调试图。
    if (result_opt && debug_enabled_) {
      publish_debug_image(msg, frame_bgr, *result_opt);
    }

    publish_cmd_vel(0.0, 0.0, true, false);
    return;
  }

  const auto now = this->now();

  double linear_speed = 0.0;
  double yaw_rate = 0.0;
  TrackState current_state = TrackState::LOST;
  bool publish_stop_cmd = false;
  bool publish_avoid_cmd = false;

  // ============================================================
  // 红色避障检测与投票（双路：实心红锥 + 镂空红禁行区；最近威胁优先 + 方向锁存）
  // ============================================================
  const bool soft_red_enabled =
    this->get_parameter("enable_soft_red_avoidance").as_bool();
  const double red_soft_yaw =
    std::max(0.0, this->get_parameter("red_soft_yaw").as_double());
  const double red_zone_soft_yaw =
    std::max(0.0, this->get_parameter("red_zone_soft_yaw").as_double());
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
  g_red_avoid.still_blocking = false;
  g_red_avoid.risk = 0.0;
  g_red_avoid.very_close = false;

  if (enable_red_filter_ && result_opt && result_opt->line_detected) {
    cv::Point2d line_centroid_roi(result_opt->centroid.x, result_opt->centroid.y);

    double side_sign = 0.0;
    cv::Point2d red_center_img, div_base_img, div_target_img;
    bool red_seen = detect_red(
      frame_bgr, *result_opt, line_centroid_roi, side_sign,
      red_center_img, div_base_img, div_target_img);

    last_red_overlay_valid_ = red_seen;

    if (red_seen) {
      last_red_center_img_ = red_center_img;
      last_div_base_img_ = div_base_img;
      last_div_target_img_ = div_target_img;

      const double red_bottom_ratio = clamp(g_red_avoid.bottom_ratio, 0.0, 1.0);
      const double eff_near_ratio = clamp(g_red_avoid.effective_near_ratio, 0.0, 1.0);
      const double obs_cx_norm = clamp(
        g_red_avoid.center_img.x / std::max(1.0, static_cast<double>(frame_bgr.cols)), 0.0, 1.0);

      // 若障碍类型切换、目标中心大幅跳变，或整个障碍框已完全越过道路中心到另一侧（如大转弯后），
      // 立即释放旧锁存，绝不允许锁存把车推向障碍所在侧。
      const double box_left_x = static_cast<double>(g_red_avoid.bbox_img.x);
      const double box_right_x = static_cast<double>(g_red_avoid.bbox_img.x + g_red_avoid.bbox_img.width);
      const double road_cx = static_cast<double>(result_opt->centroid.x);
      const bool stale_opposite_latch =
        (red_committed_dir_ > 0 && box_right_x < road_cx + 15.0) ||
        (red_committed_dir_ < 0 && box_left_x > road_cx - 15.0) ||
        (!g_red_avoid.edge_clipped && red_committed_dir_ > 0 && g_red_avoid.w_left_px < 35.0 &&
         g_red_avoid.w_right_px >= g_red_avoid.w_left_px + 60.0) ||
        (!g_red_avoid.edge_clipped && red_committed_dir_ < 0 && g_red_avoid.w_right_px < 35.0 &&
         g_red_avoid.w_left_px >= g_red_avoid.w_right_px + 60.0);
      if (red_committed_dir_ != 0 &&
          (stale_opposite_latch ||
           g_red_avoid.is_red_zone != red_latch_is_zone_ ||
           std::abs(obs_cx_norm - red_latch_cx_) > 0.28))
      {
        red_committed_dir_ = 0;
        red_left_votes_ = 0;
        red_right_votes_ = 0;
      }

      const bool hold_clipped_cone =
        (!g_red_avoid.is_red_zone && !g_red_avoid.is_red_line &&
         g_red_avoid.edge_clipped && red_committed_dir_ == 0 && red_bottom_ratio < 0.65);

      // 一旦进入近场主动避让（同目标连续跟踪且未完全越过中心线），锁存本次绕行方向，杜绝过锥中段 180° 符号翻转。
      if (soft_red_enabled && red_committed_dir_ != 0 &&
          red_bottom_ratio >= eff_near_ratio * 0.85)
      {
        side_sign = static_cast<double>(red_committed_dir_);
      }

      if (!hold_clipped_cone) {
        if (side_sign < 0.0) {
          red_left_votes_++;
          red_right_votes_ = 0;
        } else {
          red_right_votes_++;
          red_left_votes_ = 0;
        }
      }

      if (soft_red_enabled) {
        red_turning_ = false;

        const bool vote_confirmed =
          red_committed_dir_ != 0 ||
          red_left_votes_ >= red_vote_frames_ ||
          red_right_votes_ >= red_vote_frames_;

        // 用红障底部中心计算其到未分裂道路行驶方向线的横向距离。
        const cv::Point2d corridor_v = div_target_img - div_base_img;
        const cv::Point2d red_v = g_red_avoid.bottom_img - div_base_img;
        const double corridor_norm = std::hypot(corridor_v.x, corridor_v.y);

        double lateral_distance = 1e9;
        if (corridor_norm > 1e-6) {
          lateral_distance = std::abs(
            corridor_v.x * red_v.y - corridor_v.y * red_v.x) / corridor_norm;
        }

        const double corridor_limit =
          red_corridor_half_width_ratio * static_cast<double>(frame_bgr.cols);
        const bool red_in_corridor = lateral_distance <= corridor_limit;
        g_red_avoid.in_corridor = red_in_corridor;

        const bool red_close = red_bottom_ratio >= eff_near_ratio;
        const bool red_very_close = red_bottom_ratio >= red_very_close_ratio;

        // 障碍内侧边缘仍侵入车身通行宽度时才继续避让，已彻底错到角落则及时释放。
        const double frame_w = std::max(1.0, static_cast<double>(frame_bgr.cols));
        const double red_left_ratio = static_cast<double>(g_red_avoid.bbox_img.x) / frame_w;
        const double red_right_ratio =
          static_cast<double>(g_red_avoid.bbox_img.x + g_red_avoid.bbox_img.width) / frame_w;
        const double clear_margin =
          g_red_avoid.is_red_zone ? 0.33 : (g_red_avoid.is_red_line ? 0.22 : 0.14);
        const bool red_still_blocking =
          (side_sign < 0.0) ? (red_right_ratio >= clear_margin) : (red_left_ratio <= 1.0 - clear_margin);
        g_red_avoid.still_blocking = red_still_blocking;

        const double chosen_gw =
          (side_sign < 0.0) ? g_red_avoid.w_right_px : g_red_avoid.w_left_px;
        const bool narrow_cone_gap =
          (!g_red_avoid.is_red_line && !g_red_avoid.is_red_zone &&
           chosen_gw >= 35.0 && chosen_gw < 0.46 * frame_w);

        if (!hold_clipped_cone &&
            vote_confirmed &&
            red_close &&
            red_still_blocking &&
            (red_in_corridor || red_very_close || narrow_cone_gap))
        {
          soft_red_active = true;
          red_clear_frames_ = 0;

          // 进入近距主动避让区后锁存本次目标绕行方向
          const double latch_margin =
            g_red_avoid.is_red_zone ? 0.04 : (g_red_avoid.edge_clipped ? 0.15 : 0.02);
          if (red_committed_dir_ == 0 && red_bottom_ratio >= eff_near_ratio + latch_margin) {
            red_committed_dir_ = (side_sign < 0.0) ? -1 : 1;
            red_latch_cx_ = obs_cx_norm;
            red_latch_is_zone_ = g_red_avoid.is_red_zone;
          }

          double base_avoid_yaw = red_soft_yaw;
          if (g_red_avoid.is_red_line) {
            base_avoid_yaw = std::max(0.0, this->get_parameter("red_line_soft_yaw").as_double());
          } else if (g_red_avoid.is_red_zone) {
            base_avoid_yaw = red_zone_soft_yaw;
          }

          soft_red_yaw_bias =
            (side_sign < 0.0) ? -base_avoid_yaw : base_avoid_yaw;
        } else {
          red_clear_frames_++;
          if (red_clear_frames_ >= red_latch_clear_frames_) {
            red_committed_dir_ = 0;
            red_clear_frames_ = 0;
          }
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
      red_clear_frames_++;
      if (red_clear_frames_ >= red_latch_clear_frames_) {
        red_committed_dir_ = 0;
        red_clear_frames_ = 0;
      }

      if (soft_red_enabled) {
        red_turning_ = false;
      } else if (now >= red_turn_end_time_) {
        red_turning_ = false;
      }
    }
  } else {
    last_red_overlay_valid_ = false;
    g_red_avoid.valid = false;
    g_red_avoid.red_line_detected = false;
    g_red_avoid.is_red_line = false;
    g_red_avoid.is_red_zone = false;

    if (soft_red_enabled) {
      red_left_votes_ = 0;
      red_right_votes_ = 0;
      red_clear_frames_++;
      if (red_clear_frames_ >= red_latch_clear_frames_) {
        red_committed_dir_ = 0;
        red_clear_frames_ = 0;
      }
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
  } else if (yellow_line_stop_active_ &&
             since_yellow_detection < yellow_stop_duration_)
  {
    current_state = TrackState::YELLOW_LINE_DETECTED;
    linear_speed = 0.0;
    yaw_rate = 0.0;
    publish_stop_cmd = true;
  } else {
    if (yellow_line_stop_active_ &&
        since_yellow_detection >= yellow_stop_duration_)
    {
      yellow_line_stop_active_ = false;
      RCLCPP_INFO(
        this->get_logger(),
        "Yellow line stop duration ended. Resuming line following.");
    }

    if (result_opt && result_opt->line_detected) {
      const auto & result = *result_opt;
      last_detection_time_ = now;
      current_state = TrackState::TRACKING;

      double d_term = 0.0;
      if (kd_ > 0.0) {
        if (prev_err_valid_) {
          const double dt = (now - prev_err_time_).seconds();
          if (dt > 1e-3 && dt < 0.25) {
            const double raw_d = (result.error - prev_error_) / dt;
            filtered_d_err_ = 0.35 * raw_d + 0.65 * filtered_d_err_;
            d_term = kd_ * clamp(filtered_d_err_, -4.0, 4.0);
          }
        }
        prev_error_ = result.error;
        prev_err_time_ = now;
        prev_err_valid_ = true;
      }

      const double yaw_cmd =
        clamp(-(kp_ * result.error + d_term), -max_yaw_rate_, max_yaw_rate_);

      const double composite_err = std::abs(result.error);
      const double speed_scale =
        1.0 - speed_gain_damping_ * std::min(1.0, composite_err);
      double speed_cmd =
        clamp(target_speed_ * speed_scale, min_speed_, max_speed_);

      // 连续弯辅助只改变“特殊路段”的速度上限。
      // 普通弯仍保持原来的 70/30 目标点和 PD 控制。
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
      prev_err_valid_ = false;
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
  // 模式仲裁：黄色停车 > 丁字路口并线(T-merge) > 红色避障 > 交叉口探针 > 普通巡线
  // ============================================================
  const bool yellow_stop_now =
    permanent_stop_ ||
    (yellow_line_stop_active_ && since_yellow_detection < yellow_stop_duration_);

  // 仅在正常行驶（已启动且非黄停）时发布不受预瞄漂移影响的稳定质心，供数字转向使用。
  if (!yellow_stop_now && result_opt && result_opt->line_detected &&
      frame_bgr.cols > 0 && frame_bgr.rows > 0)
  {
    const double norm_x = clamp(
      result_opt->centroid.x / static_cast<double>(frame_bgr.cols), 0.0, 1.0);
    const double norm_y = clamp(
      (static_cast<double>(result_opt->roi_top) + result_opt->centroid_ref_y) /
        static_cast<double>(frame_bgr.rows),
      0.0, 1.0);
    publish_centroid(norm_x, norm_y);
  }

  // ============================================================
  // 岔路内支路 ┤ 丁字路口安全并线（底部连通白路防白墙干扰 + 里程门控 + 优先于红障防抢占冲墙）
  // ============================================================
  bool tjunction_suppresses_red = false;
  if (tjunction_armed_ && yellow_line_detection_count_ > tjunction_arm_yellow_count_) {
    tjunction_armed_ = false;
    tjunction_wall_confirm_ = 0;
  }
  if (enable_tjunction_merge_ && !yellow_stop_now &&
      (tjunction_active_ || (tjunction_armed_ && !tjunction_done_)) &&
      result_opt && !result_opt->road_mask.empty())
  {
    const cv::Mat & rmask = result_opt->road_mask;
    const int rh = rmask.rows;
    const int rw = rmask.cols;

    // 仅保留与 ROI 底部连通的白色道路连通域，彻底免疫蓝垫后方房间白墙/窗户反光！
    cv::Mat labels;
    const int num_labels = cv::connectedComponents(rmask, labels, 8, CV_32S);
    std::vector<unsigned char> keep_label(static_cast<size_t>(std::max(1, num_labels)), 0);
    const int seed_y0 = std::max(0, rh - 10);
    for (int y = seed_y0; y < rh; ++y) {
      const int * lptr = labels.ptr<int>(y);
      for (int x = 0; x < rw; ++x) {
        const int lbl = lptr[x];
        if (lbl > 0 && lbl < num_labels) {
          keep_label[static_cast<size_t>(lbl)] = 1;
        }
      }
    }
    cv::Mat bot_mask = cv::Mat::zeros(rh, rw, CV_8UC1);
    for (int y = 0; y < rh; ++y) {
      const int * lptr = labels.ptr<int>(y);
      unsigned char * bptr = bot_mask.ptr<unsigned char>(y);
      for (int x = 0; x < rw; ++x) {
        const int lbl = lptr[x];
        if (lbl > 0 && lbl < num_labels && keep_label[static_cast<size_t>(lbl)]) {
          bptr[x] = 255;
        }
      }
    }

    const int uc_y0 = static_cast<int>(rh * 0.05);
    const int uc_y1 = static_cast<int>(rh * 0.40);
    const int uc_x0 = static_cast<int>(rw * 0.25);
    const int uc_x1 = static_cast<int>(rw * 0.75);
    const cv::Mat upper_center = bot_mask(cv::Rect(
      uc_x0, uc_y0, std::max(1, uc_x1 - uc_x0), std::max(1, uc_y1 - uc_y0)));
    const double bot_upper_road =
      static_cast<double>(cv::countNonZero(upper_center)) /
      std::max(1.0, static_cast<double>(upper_center.total()));

    if (tjunction_active_) {
      const double elapsed = (now - tjunction_start_time_).seconds();
      const bool visual_aligned =
        elapsed >= tjunction_min_duration_ && bot_upper_road >= 0.28;
      const bool timed_out = elapsed >= tjunction_max_duration_;

      if (visual_aligned || timed_out) {
        tjunction_active_ = false;
        tjunction_armed_ = false;
        tjunction_done_ = true;
        tjunction_wall_confirm_ = 0;
        red_committed_dir_ = 0;
        red_left_votes_ = 0;
        red_right_votes_ = 0;
        red_clear_frames_ = 0;
        RCLCPP_INFO(
          this->get_logger(),
          "[T-merge] Completed in %.2fs (bot_upper_road=%.2f). Handing back to closed-loop.",
          elapsed, bot_upper_road);
      } else {
        red_committed_dir_ = 0;
        red_left_votes_ = 0;
        red_right_votes_ = 0;
        red_clear_frames_ = 0;
        linear_speed = tjunction_merge_speed_;
        yaw_rate = clamp(
          static_cast<double>(tjunction_merge_direction_) * tjunction_merge_yaw_,
          -max_yaw_rate_, max_yaw_rate_);
        current_state = TrackState::TRACKING;
        publish_avoid_cmd = true;
        tjunction_suppresses_red = true;
      }
    } else if (tjunction_armed_ && !tjunction_done_) {
      // 脱离岔口 2 后的真实行驶里程 / 时间门控（彻底屏蔽岔口 2 Y 型分岔 0.05s 误触发）
      const double post_dist = odom_total_dist_m_ - tjunction_arm_dist_m_;
      const double post_time = (now - tjunction_last_det_time_).seconds();
      const bool travel_ready = odom_received_ ?
        (post_dist >= tjunction_min_arm_dist_) :
        (post_time >= tjunction_min_arm_time_);

      const bool suppress_red_approach = odom_received_ ?
        (post_dist >= 0.30) :
        (post_time >= 1.0);
      if (suppress_red_approach) {
        tjunction_suppresses_red = true;
      }

      if (travel_ready) {
        const int bot_start = static_cast<int>(rh * 0.20);
        int full_rows = 0;
        int first_full_row = -1;
        for (int y = bot_start; y < rh; ++y) {
          const int white_cnt = cv::countNonZero(bot_mask.row(y));
          if (static_cast<double>(white_cnt) >= static_cast<double>(rw) * 0.78) {
            if (first_full_row < 0) {
              first_full_row = y;
            }
            full_rows++;
          }
        }

        // 顶部暗带判定（取 ROI 顶部 0..18% 的全宽区域），对 ±20° 斜角接近丁字墙完全免疫；
        // 旧版在 first_full_row 上方紧贴切矩形，只要车身斜 5° 就会把倾斜白路切进矩形导致 bot_top_dark 掉到 0.52 漏触发。
        const int top_strip_h = std::max(8, static_cast<int>(rh * 0.18));
        const cv::Mat top_strip = bot_mask(cv::Rect(0, 0, rw, std::min(top_strip_h, rh)));
        const double top_strip_dark = 1.0 - static_cast<double>(cv::countNonZero(top_strip)) /
          std::max(1.0, static_cast<double>(top_strip.total()));

        double bot_top_dark = top_strip_dark;
        if (first_full_row > 8) {
          const int wall_h = std::min(first_full_row, std::max(12, static_cast<int>(rh * 0.25)));
          const int wall_y0 = std::max(0, first_full_row - wall_h);
          const cv::Mat wall_band = bot_mask(cv::Rect(0, wall_y0, rw, first_full_row - wall_y0));
          const double local_dark = 1.0 - static_cast<double>(cv::countNonZero(wall_band)) /
            std::max(1.0, static_cast<double>(wall_band.total()));
          bot_top_dark = std::max(top_strip_dark, local_dark);
        }

        // 真实 ┤ 丁字墙特征：底部满宽横贯 + 顶部暗墙封顶(>=0.82) + 前方无直行道路(bot_upper_road<=0.35)
        if (full_rows >= tjunction_min_full_rows_ &&
            bot_top_dark >= 0.75 &&
            bot_upper_road <= 0.42)
        {
          tjunction_wall_confirm_++;
          // 在确认 T 墙期间禁止右侧红禁行区把车向左推过黑墙！
          tjunction_suppresses_red = true;
          if (tjunction_wall_confirm_ >= 1) {
            tjunction_active_ = true;
            tjunction_start_time_ = now;
            red_committed_dir_ = 0;
            linear_speed = tjunction_merge_speed_;
            yaw_rate = clamp(
              static_cast<double>(tjunction_merge_direction_) * tjunction_merge_yaw_,
              -max_yaw_rate_, max_yaw_rate_);
            current_state = TrackState::TRACKING;
            publish_avoid_cmd = true;
            RCLCPP_INFO(
              this->get_logger(),
              "[T-merge] Horizontal T-wall confirmed (full_rows=%d, top_dark=%.2f, upper=%.2f, dist=%.2fm). "
              "Merging %s at v=%.2f, w=%.2f.",
              full_rows, bot_top_dark, bot_upper_road, post_dist,
              tjunction_merge_direction_ < 0 ? "RIGHT" : "LEFT",
              linear_speed, yaw_rate);
          }
        } else {
          tjunction_wall_confirm_ = std::max(0, tjunction_wall_confirm_ - 1);
        }
      }
    }
  }

  if (!yellow_stop_now && !tjunction_active_ && intersection_active_) {
    if (now <= intersection_end_time_) {
      linear_speed = intersection_turn_speed_;
      yaw_rate = clamp(intersection_turn_yaw_rate_, -max_yaw_rate_, max_yaw_rate_);
      current_state = TrackState::TRACKING;
    } else {
      intersection_active_ = false;
    }
  }

  if (!yellow_stop_now && !tjunction_active_ && soft_red_enabled) {
    // 若正在确认 T 墙且红障企图往逆行/撞墙侧推，抑制该反向避障
    const bool blocked_by_tmerge =
      tjunction_suppresses_red &&
      (soft_red_yaw_bias * static_cast<double>(tjunction_merge_direction_) < 0.0);

    if (soft_red_active && !blocked_by_tmerge) {
      const double bottom_ratio =
        clamp(g_red_avoid.bottom_ratio, 0.0, 1.0);
      const double eff_near = clamp(g_red_avoid.effective_near_ratio, 0.0, 0.95);
      const double risk_denominator =
        std::max(1e-3, red_very_close_ratio - eff_near);

      const double risk =
        clamp((bottom_ratio - eff_near) / risk_denominator, 0.0, 1.0);
      g_red_avoid.risk = risk;

      const double repulsion_gain =
        std::max(0.0, this->get_parameter("red_repulsion_gain").as_double());
      const double lane_keep_min_factor =
        clamp(this->get_parameter("red_lane_keep_min_factor").as_double(), 0.0, 1.0);

      const bool red_on_left = soft_red_yaw_bias < 0.0;

      double repulsion_strength =
        std::abs(soft_red_yaw_bias) * (1.0 + repulsion_gain * risk);

      if (g_curve_assist.reverse_curve_active) {
        const double curve_bias_limit =
          clamp(this->get_parameter("red_soft_curve_yaw_limit").as_double(), 0.0, max_yaw_rate_);
        const double dynamic_limit =
          curve_bias_limit + risk * (max_yaw_rate_ - curve_bias_limit);
        repulsion_strength = std::min(repulsion_strength, dynamic_limit);
      }

      const double lane_keep_factor =
        1.0 - (1.0 - lane_keep_min_factor) * risk;

      const double edge_guard_start = clamp(
        this->get_parameter("red_edge_guard_start_error").as_double(), 0.0, 0.9);
      const double edge_guard_full = clamp(
        this->get_parameter("red_edge_guard_full_error").as_double(),
        edge_guard_start + 0.02, 1.0);
      const double edge_guard_min_rep = clamp(
        this->get_parameter("red_edge_guard_min_repulsion").as_double(), 0.0, 1.0);
      const double line_err =
        (result_opt && result_opt->line_detected) ? result_opt->error : 0.0;
      const double edge_err =
        red_on_left ? std::max(0.0, -line_err) : std::max(0.0, line_err);
      const double edge_guard = clamp(
        (edge_err - edge_guard_start) / (edge_guard_full - edge_guard_start),
        0.0, 1.0);

      // 极近距离保留最低斥力下限，避免贴锥前完全撤力导致撞锥
      const double min_keep = (bottom_ratio >= red_very_close_ratio) ? edge_guard_min_rep : 0.0;
      repulsion_strength *= std::max(min_keep, 1.0 - edge_guard);
      const double effective_lane_keep_factor =
        lane_keep_factor + (1.0 - lane_keep_factor) * edge_guard;

      const double chosen_gl =
        red_on_left ? g_red_avoid.gap_right_l : g_red_avoid.gap_left_l;
      const double chosen_gr =
        red_on_left ? g_red_avoid.gap_right_r : g_red_avoid.gap_left_r;
      const double chosen_gw =
        red_on_left ? g_red_avoid.w_right_px : g_red_avoid.w_left_px;

      if (!g_red_avoid.is_red_line && chosen_gw >= 35.0) {
        const double rw = std::max(1.0, static_cast<double>(frame_bgr.cols));
        const double half_w = 0.5 * rw;
        const double margin_px = (g_red_avoid.is_red_zone ? 0.17 : 0.23) * rw;
        const double normal_x = (result_opt && result_opt->line_detected) ?
          static_cast<double>(result_opt->centroid.x) : half_w;
        const double lo = chosen_gl + margin_px;
        const double hi = chosen_gr - margin_px;
        const double gap_target_x = (lo <= hi) ?
          clamp(normal_x, lo, hi) :
          0.5 * (chosen_gl + chosen_gr);
        const double gap_err = clamp((gap_target_x - half_w) / half_w, -1.0, 1.0);
        double gap_yaw = -kp_ * gap_err;
        if (g_red_avoid.is_red_zone) {
          gap_yaw = clamp(gap_yaw, -0.38, 0.38);
        } else {
          gap_yaw = clamp(gap_yaw, -1.30, 1.30);
        }
        yaw_rate = clamp(gap_yaw, -max_yaw_rate_, max_yaw_rate_);
      } else {
        if (!g_red_avoid.is_red_line && !g_red_avoid.is_red_zone && red_committed_dir_ != 0) {
          repulsion_strength = std::max(repulsion_strength, 0.85);
        }
        if (red_on_left) {
          if (yaw_rate > 0.0) {
            yaw_rate = yaw_rate * effective_lane_keep_factor - repulsion_strength;
          } else {
            yaw_rate = std::min(yaw_rate, -repulsion_strength);
          }
        } else {
          if (yaw_rate < 0.0) {
            yaw_rate = yaw_rate * effective_lane_keep_factor + repulsion_strength;
          } else {
            yaw_rate = std::max(yaw_rate, repulsion_strength);
          }
        }
        if (red_on_left && edge_err >= 0.30) {
          yaw_rate = std::max(yaw_rate, -0.50);
        } else if (!red_on_left && edge_err >= 0.30) {
          yaw_rate = std::min(yaw_rate, 0.50);
        }
        const double avoid_cap = std::min(1.05, max_yaw_rate_);
        yaw_rate = clamp(yaw_rate, -avoid_cap, avoid_cap);
      }

      const double red_base_speed =
        g_red_avoid.is_red_line ?
        std::max(0.0, this->get_parameter("red_line_speed").as_double()) :
        red_soft_speed;

      double safe_speed =
        std::max(red_min_forward_speed, red_base_speed * (1.0 - 0.60 * risk));
      if (!g_red_avoid.is_red_line) {
        if (g_red_avoid.is_red_zone) {
          safe_speed = std::max(0.18, red_base_speed * (1.0 - 0.20 * risk));
        } else {
          safe_speed = std::max(0.14, red_base_speed * (1.0 - 0.38 * risk));
        }
      }

      linear_speed =
        std::max(red_min_forward_speed, std::min(linear_speed, safe_speed));

      g_red_avoid.very_close = bottom_ratio >= red_very_close_ratio;
      current_state = TrackState::TRACKING;
      publish_avoid_cmd = true;
    }
  } else if (!yellow_stop_now && !tjunction_active_ && red_turning_ && now < red_turn_end_time_) {
    linear_speed = std::max(red_min_forward_speed, red_turn_linear_);
    yaw_rate = red_turn_yaw_cmd_;
    current_state = TrackState::TRACKING;
    publish_avoid_cmd = true;
  } else if (red_turning_ && now >= red_turn_end_time_) {
    red_turning_ = false;
  }

  // ============================================================
  // 最终安全锁：黄色停车拥有最高优先级，任何后续控制都不能覆盖
  // ============================================================
  if (permanent_stop_ ||
      (yellow_line_stop_active_ && since_yellow_detection < yellow_stop_duration_))
  {
    current_state = TrackState::YELLOW_LINE_DETECTED;
    linear_speed = 0.0;
    yaw_rate = 0.0;
    publish_stop_cmd = true;
    publish_avoid_cmd = false;
    red_committed_dir_ = 0;
  }

  // 只有黄色停车锁可以把运行中的车速压到 0。
  if (!yellow_stop_now && linear_speed > 0.0) {
    g_motion.valid = true;
    g_motion.last_linear = linear_speed;
    g_motion.last_yaw = yaw_rate;
  }

  if (result_opt && debug_enabled_) {
    publish_debug_image(msg, frame_bgr, *result_opt);
  }

  // 岔路实验模式限制正常巡线转向指令的变化率；停车、丢线、避障、T-merge 均绕过。
  const bool curve_preview_enabled =
    this->get_parameter("enable_curve_preview_steering").as_bool();
  if (curve_preview_enabled && current_state == TrackState::TRACKING &&
      result_opt && result_opt->line_detected && !publish_stop_cmd &&
      !publish_avoid_cmd && !reverse_brake_active_ && !red_turning_)
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
    publish_cmd_vel(linear_speed, yaw_rate, publish_stop_cmd, publish_avoid_cmd);
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

    const int yellow_pixels = cv::countNonZero(yellow_mask);
    const double yellow_area_ratio =
      yellow_mask.total() > 0 ?
      static_cast<double>(yellow_pixels) /
        static_cast<double>(yellow_mask.total()) :
      0.0;

    yellow_line_detected =
      yellow_area_ratio >= yellow_area_ratio_threshold_;
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
  // 5. 形态学：先去小噪声，再补小断裂 + 内部红障水平桥接（防红锥劈路导致质心瞬移）
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

  cv::Mat red_mask_raw;
  cv::Mat road_mask = binary.clone();
  if (enable_red_filter_) {
    // 使用未高斯模糊的 ROI 计算原始红色掩码，保留 1cm 细红禁行区条纹
    cv::Mat hsv_raw;
    cv::cvtColor(roi, hsv_raw, cv::COLOR_BGR2HSV);
    cv::Mat r1, r2;
    cv::inRange(
      hsv_raw,
      cv::Scalar(static_cast<double>(red_h_low_1_), static_cast<double>(red_s_min_), static_cast<double>(red_v_min_)),
      cv::Scalar(static_cast<double>(red_h_high_1_), 255.0, 255.0),
      r1);
    cv::inRange(
      hsv_raw,
      cv::Scalar(static_cast<double>(red_h_low_2_), static_cast<double>(red_s_min_), static_cast<double>(red_v_min_)),
      cv::Scalar(static_cast<double>(red_h_high_2_), 255.0, 255.0),
      r2);
    cv::bitwise_or(r1, r2, red_mask_raw);

    // 轻微膨胀仅用于弥合锥桶边缘与白路间的抗锯齿缝隙
    cv::Mat red_dil;
    const cv::Mat k5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(red_mask_raw, red_dil, k5);

    // 按行桥接：仅当红色区段左右两侧均紧邻白路（或一侧贴图边且另一侧有大块白路）时
    // 才在 road_mask 中补洞，彻底阻止红锥把单条白路劈成两半，同时绝不把赛道外红锥并入白路。
    const int kSearchMargin = 14;
    const int kBorderWhiteReq = 32;
    for (int y = 0; y < road_mask.rows; ++y) {
      const unsigned char * r_ptr = red_dil.ptr<unsigned char>(y);
      unsigned char * w_ptr = road_mask.ptr<unsigned char>(y);
      int x = 0;
      while (x < road_mask.cols) {
        if (r_ptr[x] == 0) {
          ++x;
          continue;
        }
        const int rx0 = x;
        while (x < road_mask.cols && r_ptr[x] != 0) {
          ++x;
        }
        const int rx1 = x - 1;

        bool white_left = false;
        const int lx0 = std::max(0, rx0 - kSearchMargin);
        for (int lx = lx0; lx < rx0; ++lx) {
          if (w_ptr[lx] != 0) {
            white_left = true;
            break;
          }
        }
        bool white_right = false;
        const int rx_end = std::min(road_mask.cols - 1, rx1 + kSearchMargin);
        for (int rx = rx1 + 1; rx <= rx_end; ++rx) {
          if (w_ptr[rx] != 0) {
            white_right = true;
            break;
          }
        }

        bool should_bridge = (white_left && white_right);
        if (!should_bridge && rx0 <= 2 && white_right) {
          int cnt = 0;
          const int chk_end = std::min(road_mask.cols - 1, rx1 + kBorderWhiteReq + kSearchMargin);
          for (int rx = rx1 + 1; rx <= chk_end; ++rx) {
            if (w_ptr[rx] != 0) {
              cnt++;
            }
          }
          should_bridge = (cnt >= kBorderWhiteReq);
        } else if (!should_bridge && rx1 >= road_mask.cols - 3 && white_left) {
          int cnt = 0;
          const int chk_start = std::max(0, rx0 - kBorderWhiteReq - kSearchMargin);
          for (int lx = chk_start; lx < rx0; ++lx) {
            if (w_ptr[lx] != 0) {
              cnt++;
            }
          }
          should_bridge = (cnt >= kBorderWhiteReq);
        }

        if (should_bridge) {
          for (int bx = rx0; bx <= rx1; ++bx) {
            w_ptr[bx] = 255;
          }
        }
      }
    }
  }

  ProcessingResult result{};
  result.line_detected = false;
  result.yellow_line_detected = yellow_line_detected;
  result.roi_bgr = roi.clone();
  result.binary_mask = binary.clone();
  result.road_mask = road_mask;
  result.red_mask_raw = red_mask_raw;
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
  // 7. 局部前视白色通道中心（基于遮挡补洞后的 road_mask，彻底避免红锥劈路跳变）
  // ============================================================
  struct BandRun
  {
    double center{0.0};
    int left{0};
    int right{0};
    int width{0};
  };

  auto find_band_run =
    [&](int y0, int y1, double reference_x) -> std::optional<BandRun>
    {
      y0 = std::clamp(y0, 0, road_mask.rows - 1);
      y1 = std::clamp(y1, y0 + 1, road_mask.rows);

      const cv::Mat band =
        road_mask(cv::Rect(0, y0, road_mask.cols, y1 - y0));

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

      std::vector<unsigned char> active(road_mask.cols, 0);

      for (int x = 0; x < road_mask.cols; ++x) {
        if (cv::countNonZero(band.col(x)) >= min_white_pixels) {
          active[x] = 1;
        }
      }

      // 太窄的连续白色区域视为反光/噪声。
      constexpr int kMinRunWidth = 20;

      bool found = false;
      BandRun best{};
      double best_distance = 1e9;
      int run_start = -1;

      for (int x = 0; x <= road_mask.cols; ++x) {
        const bool is_active =
          (x < road_mask.cols) ? (active[x] != 0) : false;

        if (is_active && run_start < 0) {
          run_start = x;
        }

        if ((!is_active || x == road_mask.cols) && run_start >= 0) {
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
                 run_width > best.width))
            {
              found = true;
              best.center = center;
              best.left = run_start;
              best.right = run_end;
              best.width = run_width;
              best_distance = distance;
            }
          }

          run_start = -1;
        }
      }

      if (!found) {
        return std::nullopt;
      }

      return best;
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

  const auto near_run = find_band_run(near_y0, near_y1, image_center_x);
  const std::optional<double> near_center =
    near_run.has_value() ? std::optional<double>(near_run->center) : std::nullopt;

  if (near_run.has_value()) {
    result.near_road_valid = true;
    result.near_road_left = near_run->left;
    result.near_road_right = near_run->right;
    result.narrow_lane_visible =
      (near_run->width >= 40) &&
      (static_cast<double>(near_run->width) <= static_cast<double>(width) * 0.78);
  }

  const double far_reference =
    near_center.has_value() ? near_center.value() : image_center_x;

  const auto far_run = find_band_run(far_y0, far_y1, far_reference);
  const std::optional<double> far_center =
    far_run.has_value() ? std::optional<double>(far_run->center) : std::nullopt;

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
    const double base_far_weight = clamp(
      this->get_parameter("base_far_weight").as_double(), 0.0, 0.50);
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
        this->get_parameter("curve_preview_max_far_weight").as_double(), base_far_weight, 0.65);
      double far_weight = base_far_weight;
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
        far_weight += (max_far_weight - base_far_weight) * anticipation;
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
      // Default base_far_weight=0.30 preserves exact original 70% near + 30% far.
      target_x = (1.0 - base_far_weight) * near_center.value() +
        base_far_weight * far_center.value();

      const double near_y =
        0.5 * static_cast<double>(near_y0 + near_y1);
      const double far_y =
        0.5 * static_cast<double>(far_y0 + far_y1);

      target_y = (1.0 - base_far_weight) * near_y + base_far_weight * far_y;
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
  // 固定纵向参考线（70% near + 30% far），彻底与 curve_preview 动态远端权重解耦。
  result.centroid_ref_y =
    0.70 * (0.5 * static_cast<double>(near_y0 + near_y1)) +
    0.30 * (0.5 * static_cast<double>(far_y0 + far_y1));

  result.error =
    (target_x - image_center_x) / image_center_x;
  result.error = clamp(result.error, -1.0, 1.0);
  result.state = TrackState::TRACKING;

  return result;
}

bool LineFollowerNode::detect_red(
  const cv::Mat & frame_bgr, const ProcessingResult & result,
  const cv::Point2d & line_centroid_roi, double & side_sign,
  cv::Point2d & red_centroid_img, cv::Point2d & div_base_img,
  cv::Point2d & div_target_img) const
{
  side_sign = 0.0;

  // 每次检测先清空当前帧结果。
  g_red_avoid = RedAvoidRuntime{};

  if (!enable_red_filter_ || frame_bgr.empty() || result.red_mask_raw.empty()) {
    return false;
  }

  const int roi_top = std::clamp(result.roi_top, 0, frame_bgr.rows - 1);
  const int roi_bottom = std::clamp(result.roi_bottom, roi_top + 1, frame_bgr.rows);
  const cv::Mat & mask_raw = result.red_mask_raw;
  const int roi_h = std::max(1, mask_raw.rows);
  const int roi_w = std::max(1, mask_raw.cols);
  const double roi_area = std::max(1.0, static_cast<double>(mask_raw.total()));

  const double line_aspect_min =
    std::max(1.0, this->get_parameter("red_line_aspect_ratio_min").as_double());
  const double line_width_ratio_min =
    clamp(this->get_parameter("red_line_min_width_ratio").as_double(), 0.0, 1.0);
  const double cone_area_ratio_min =
    std::max(0.0, this->get_parameter("red_cone_area_ratio_threshold").as_double());
  const int cone_min_bbox_width =
    std::max(1, static_cast<int>(this->get_parameter("red_cone_min_bbox_width").as_int()));
  const int cone_min_bbox_height =
    std::max(1, static_cast<int>(this->get_parameter("red_cone_min_bbox_height").as_int()));
  const bool enable_zone_cluster =
    this->get_parameter("enable_red_zone_cluster").as_bool();
  const double zone_min_width_ratio =
    clamp(this->get_parameter("red_zone_min_width_ratio").as_double(), 0.04, 0.9);
  const int zone_min_pixels =
    std::max(20, static_cast<int>(this->get_parameter("red_zone_min_pixels").as_int()));
  const double cone_near_ratio =
    clamp(this->get_parameter("red_near_ratio").as_double(), 0.0, 1.0);
  const double zone_near_ratio =
    clamp(this->get_parameter("red_zone_near_ratio").as_double(), 0.0, 1.0);

  struct RedCandidate
  {
    cv::Rect box;
    double area{0.0};
    double bottom_ratio{0.0};
    double eff_near_ratio{0.38};
    bool is_line{false};
    bool is_zone{false};
  };
  std::vector<RedCandidate> candidates;

  // ============================================================
  // 路径 A：实心红锥 / 横向红线（3x3 开闭运算去散点）
  // ============================================================
  const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat mask_cone;
  cv::morphologyEx(mask_raw, mask_cone, cv::MORPH_OPEN, k3, cv::Point(-1, -1), 1);
  cv::morphologyEx(mask_cone, mask_cone, cv::MORPH_CLOSE, k3, cv::Point(-1, -1), 1);

  std::vector<std::vector<cv::Point>> cone_contours;
  cv::findContours(mask_cone, cone_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double best_line_area = 0.0;
  cv::Rect best_line_box{};
  for (const auto & c : cone_contours) {
    const double area = cv::contourArea(c);
    if (area <= 0.0) {
      continue;
    }
    const cv::Rect box = cv::boundingRect(c);
    if (box.width <= 0 || box.height <= 0) {
      continue;
    }
    const double aspect = static_cast<double>(box.width) / static_cast<double>(box.height);
    const bool looks_like_line =
      aspect >= line_aspect_min &&
      static_cast<double>(box.width) >= static_cast<double>(frame_bgr.cols) * line_width_ratio_min;

    if (looks_like_line) {
      if (area > best_line_area) {
        best_line_area = area;
        best_line_box = box;
      }
      candidates.push_back(RedCandidate{
        box, area,
        clamp(static_cast<double>(box.y + box.height) / static_cast<double>(roi_h), 0.0, 1.0),
        zone_near_ratio, true, false});
      continue;
    }

    if (box.width < cone_min_bbox_width || box.height < cone_min_bbox_height) {
      continue;
    }
    if ((area / roi_area) < cone_area_ratio_min) {
      continue;
    }
    const int raw_px = cv::countNonZero(mask_raw(box));
    const double raw_fill =
      static_cast<double>(raw_px) / std::max(1.0, static_cast<double>(box.area()));
    const bool hollow_zone =
      (raw_fill < 0.28) ||
      (aspect >= 1.90 && raw_fill < 0.45 &&
       static_cast<double>(box.width) >= static_cast<double>(roi_w) * zone_min_width_ratio);
    if (hollow_zone) {
      if (enable_zone_cluster && raw_px >= zone_min_pixels && box.height >= 10 &&
          static_cast<double>(box.width) >= static_cast<double>(roi_w) * zone_min_width_ratio)
      {
        candidates.push_back(RedCandidate{
          box, static_cast<double>(raw_px),
          clamp(static_cast<double>(box.y + box.height) / static_cast<double>(roi_h), 0.0, 1.0),
          zone_near_ratio, false, true});
      }
      continue;
    }
    candidates.push_back(RedCandidate{
      box, area,
      clamp(static_cast<double>(box.y + box.height) / static_cast<double>(roi_h), 0.0, 1.0),
      cone_near_ratio, false, false});
  }

  if (best_line_area > 0.0) {
    g_red_avoid.red_line_detected = true;
    g_red_avoid.red_line_bbox_img = cv::Rect(
      best_line_box.x, best_line_box.y + roi_top, best_line_box.width, best_line_box.height);
  }

  // ============================================================
  // 路径 B：镂空红色禁行区（24x13cm 细红边框+竖条纹，27x11 闭运算聚类，避免被开运算打碎）
  // ============================================================
  if (enable_zone_cluster) {
    const cv::Mat k_zone = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(27, 11));
    cv::Mat mask_zone;
    cv::morphologyEx(mask_raw, mask_zone, cv::MORPH_CLOSE, k_zone, cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> zone_contours;
    cv::findContours(mask_zone, zone_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto & c : zone_contours) {
      const cv::Rect box = cv::boundingRect(c);
      if (static_cast<double>(box.width) < static_cast<double>(roi_w) * zone_min_width_ratio ||
          box.height < 10)
      {
        continue;
      }
      const int raw_px = cv::countNonZero(mask_raw(box));
      if (raw_px < zone_min_pixels) {
        continue;
      }
      const double fill = static_cast<double>(raw_px) /
        std::max(1.0, static_cast<double>(box.width * box.height));
      // 镂空禁行区呈明显空心条纹填充率（实心锥桶通常 > 0.72 且已由路径 A 检出）
      if (fill < 0.04 || fill > 0.72) {
        continue;
      }
      // 跳过与已有实心锥桶高度重合的框；反之若本禁行区完整包住小条纹碎片，则移除那些碎片。
      bool overlaps_solid_cone = false;
      for (const auto & existing : candidates) {
        if (!existing.is_line) {
          const cv::Rect inter = existing.box & box;
          const double ref_area = existing.is_zone ?
            static_cast<double>(std::min(box.area(), existing.box.area())) :
            static_cast<double>(box.area());
          if (inter.area() > 0 &&
              static_cast<double>(inter.area()) >= 0.55 * ref_area)
          {
            overlaps_solid_cone = true;
            break;
          }
        }
      }
      if (overlaps_solid_cone) {
        continue;
      }
      candidates.erase(
        std::remove_if(
          candidates.begin(), candidates.end(),
          [&](const RedCandidate & existing) {
            if (existing.is_zone || existing.is_line) {
              return false;
            }
            const cv::Rect inter = existing.box & box;
            return inter.area() > 0 &&
                   static_cast<double>(inter.area()) >= 0.65 * static_cast<double>(existing.box.area()) &&
                   static_cast<double>(box.width) >= 1.5 * static_cast<double>(existing.box.width);
          }),
        candidates.end());
      candidates.push_back(RedCandidate{
        box, static_cast<double>(raw_px),
        clamp(static_cast<double>(box.y + box.height) / static_cast<double>(roi_h), 0.0, 1.0),
        zone_near_ratio, false, true});
    }
  }

  if (candidates.empty()) {
    return false;
  }

  // ============================================================
  // 按最近威胁（bottom_ratio 最大）选择主目标：近处禁行区绝不被远处红锥遮蔽
  // ============================================================
  const auto chosen = *std::max_element(
    candidates.begin(), candidates.end(),
    [](const RedCandidate & a, const RedCandidate & b) {
      const bool a_near = a.bottom_ratio >= a.eff_near_ratio;
      const bool b_near = b.bottom_ratio >= b.eff_near_ratio;
      if (a_near != b_near) {
        return !a_near && b_near;
      }
      if (std::abs(a.bottom_ratio - b.bottom_ratio) > 0.03) {
        return a.bottom_ratio < b.bottom_ratio;
      }
      return a.area < b.area;
    });

  const cv::Rect box = chosen.box;
  const bool use_red_line = chosen.is_line;
  const bool use_red_zone = chosen.is_zone;

  const double aspect_ratio =
    static_cast<double>(box.width) / static_cast<double>(std::max(1, box.height));
  const double contour_area_ratio = chosen.area / roi_area;

  const cv::Point2d red_centroid_roi(
    static_cast<double>(box.x) + static_cast<double>(box.width) * 0.5,
    static_cast<double>(box.y) + static_cast<double>(box.height) * 0.5);
  const cv::Point2d red_bottom_roi(
    static_cast<double>(box.x) + static_cast<double>(box.width) * 0.5,
    static_cast<double>(box.y + box.height));

  // 以 ROI 底部中心为根点，连向未分裂道路中心目标，构建行驶参考线。
  cv::Point2d base(
    static_cast<double>(roi_w) * 0.5,
    static_cast<double>(roi_h));
  cv::Point2d v_line = line_centroid_roi - base;

  if (red_divider_angle_rad_ != 0.0) {
    const double angle =
      v_line.x >= 0.0 ? red_divider_angle_rad_ : -red_divider_angle_rad_;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    v_line = cv::Point2d(
      v_line.x * c - v_line.y * s,
      v_line.x * s + v_line.y * c);
  }

  const cv::Point2d v_red = red_centroid_roi - base;
  const double v_line_norm = std::max(1e-6, std::hypot(v_line.x, v_line.y));
  const double cross = v_line.x * v_red.y - v_line.y * v_red.x;
  const double signed_lat_px = cross / v_line_norm;

  g_red_avoid.edge_clipped =
    (!use_red_line && !use_red_zone &&
     (box.x <= 12 || (box.x + box.width) >= roi_w - 12));

  // Measure contiguous unbridged white passable gap to the left & right of obstacle.
  if (!result.binary_mask.empty()) {
    int by0 = 0;
    int by1 = 1;
    if (use_red_zone || use_red_line) {
      by1 = std::clamp(box.y + box.height + 6, 1, roi_h);
      const int band_h = std::clamp(box.height / 3, 18, 36);
      by0 = std::clamp(by1 - band_h, 0, by1 - 1);
    } else {
      by0 = std::clamp(box.y + static_cast<int>(static_cast<double>(box.height) * 0.55), 0, roi_h - 2);
      by1 = std::clamp(
        std::max(by0 + 8, box.y + static_cast<int>(static_cast<double>(box.height) * 0.85)),
        by0 + 1, roi_h);
    }
    const cv::Mat base_band = result.binary_mask(cv::Rect(0, by0, roi_w, by1 - by0));
    const double req_ratio = (use_red_zone || use_red_line) ? 0.30 : 0.25;
    const int col_req = std::max(1, static_cast<int>(std::ceil(static_cast<double>(base_band.rows) * req_ratio)));

    auto is_col_white = [&](int cx) -> bool {
      return cv::countNonZero(base_band.col(cx)) >= col_req;
    };

    auto measure_side_gap = [&](bool pass_right, double & out_l, double & out_r) -> double {
      if (pass_right) {
        const int gl = std::clamp(box.x + box.width, 0, roi_w - 1);
        int sx = gl;
        const int sx_lim = std::min(roi_w, gl + 14);
        while (sx < sx_lim && !is_col_white(sx)) {
          sx++;
        }
        if (sx >= sx_lim) {
          out_l = static_cast<double>(gl);
          out_r = static_cast<double>(gl);
          return 0.0;
        }
        int last_good = sx;
        int miss = 0;
        for (int cx = sx; cx < roi_w; ++cx) {
          if (is_col_white(cx)) {
            last_good = cx;
            miss = 0;
          } else {
            miss++;
            if (miss >= 5) {
              break;
            }
          }
        }
        out_l = static_cast<double>(gl);
        out_r = static_cast<double>(last_good);
        return std::max(0.0, out_r - out_l);
      } else {
        const int gr = std::clamp(box.x, 0, roi_w - 1);
        int sx = gr - 1;
        const int sx_lim = std::max(0, gr - 14);
        while (sx >= sx_lim && !is_col_white(sx)) {
          sx--;
        }
        if (sx < sx_lim) {
          out_l = static_cast<double>(gr);
          out_r = static_cast<double>(gr);
          return 0.0;
        }
        int last_good = sx;
        int miss = 0;
        for (int cx = sx; cx >= 0; --cx) {
          if (is_col_white(cx)) {
            last_good = cx;
            miss = 0;
          } else {
            miss++;
            if (miss >= 5) {
              break;
            }
          }
        }
        out_l = static_cast<double>(last_good);
        out_r = static_cast<double>(gr);
        return std::max(0.0, out_r - out_l);
      }
    };

    g_red_avoid.w_left_px = measure_side_gap(false, g_red_avoid.gap_left_l, g_red_avoid.gap_left_r);
    g_red_avoid.w_right_px = measure_side_gap(true, g_red_avoid.gap_right_l, g_red_avoid.gap_right_r);
  }

  // Clean centerline cross-product + |signed_lat_px| < 25px horizontal clearance tie-breaker
  if (!use_red_line &&
      std::max(g_red_avoid.w_left_px, g_red_avoid.w_right_px) >= 35.0 &&
      std::abs(g_red_avoid.w_right_px - g_red_avoid.w_left_px) >= 18.0)
  {
    side_sign = (g_red_avoid.w_right_px > g_red_avoid.w_left_px) ? -1.0 : 1.0;
  } else if (use_red_line || std::abs(signed_lat_px) < 25.0) {
    bool clearance_decided = false;
    if (!result.road_mask.empty()) {
      const int oy0 = std::clamp(box.y, 0, roi_h - 1);
      const int oy1 = std::clamp(box.y + box.height, oy0 + 1, roi_h);
      const cv::Mat oband = result.road_mask(cv::Rect(0, oy0, roi_w, oy1 - oy0));
      const int min_px = std::max(1, static_cast<int>(static_cast<double>(oband.rows) * 0.25));
      int road_l = -1;
      int road_r = -1;
      for (int x = 0; x < roi_w; ++x) {
        if (cv::countNonZero(oband.col(x)) >= min_px) {
          if (road_l < 0) {
            road_l = x;
          }
          road_r = x;
        }
      }
      if (road_l >= 0 && (road_r - road_l) >= 40) {
        const double left_clear = std::max(0.0, static_cast<double>(box.x - road_l));
        const double right_clear = std::max(0.0, static_cast<double>(road_r - (box.x + box.width)));
        if (std::abs(right_clear - left_clear) >= 16.0) {
          side_sign = (right_clear > left_clear) ? -1.0 : 1.0;
          clearance_decided = true;
        }
      }
    }
    if (!clearance_decided) {
      const double left_space = static_cast<double>(box.x);
      const double right_space = static_cast<double>(roi_w - (box.x + box.width));
      if (right_space > left_space + 10.0) {
        side_sign = -1.0;
      } else if (left_space > right_space + 10.0) {
        side_sign = 1.0;
      } else {
        const int fallback_direction =
          static_cast<int>(this->get_parameter("red_line_fallback_direction").as_int());
        side_sign = fallback_direction >= 0 ? 1.0 : -1.0;
      }
    }
  } else {
    side_sign = cross >= 0.0 ? 1.0 : -1.0;
  }

  red_centroid_img = cv::Point2d(
    red_centroid_roi.x, red_centroid_roi.y + static_cast<double>(roi_top));
  const cv::Point2d red_bottom_img(
    red_bottom_roi.x, red_bottom_roi.y + static_cast<double>(roi_top));
  div_base_img = cv::Point2d(
    base.x, base.y + static_cast<double>(roi_top));
  const cv::Point2d rotated_target = base + v_line;
  div_target_img = cv::Point2d(
    rotated_target.x, rotated_target.y + static_cast<double>(roi_top));

  g_red_avoid.valid = true;
  g_red_avoid.is_red_line = use_red_line;
  g_red_avoid.is_red_zone = use_red_zone;
  g_red_avoid.bbox_img = cv::Rect(box.x, box.y + roi_top, box.width, box.height);
  g_red_avoid.center_img = red_centroid_img;
  g_red_avoid.bottom_img = red_bottom_img;
  g_red_avoid.aspect_ratio = aspect_ratio;
  g_red_avoid.contour_area_ratio = contour_area_ratio;
  g_red_avoid.bottom_ratio = chosen.bottom_ratio;
  g_red_avoid.effective_near_ratio = chosen.eff_near_ratio;

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

void LineFollowerNode::publish_cmd_vel(
  double linear_speed, double yaw_rate, bool publish_stop, bool publish_avoid)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed;
  cmd.angular.z = yaw_rate;
  cmd_vel_pub_->publish(cmd);

  // Yellow stop topic has highest mux priority. Reverse braking also uses it so
  // the temporary pulse cannot be overwritten by the normal follower command.
  if (stop_pub_ && publish_stop) {
    stop_pub_->publish(cmd);
  } else if (avoid_pub_ && publish_avoid) {
    avoid_pub_->publish(cmd);
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
        g_red_avoid.is_red_line ? "RED_LINE" : (g_red_avoid.is_red_zone ? "RED_ZONE" : "CONE"),
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

