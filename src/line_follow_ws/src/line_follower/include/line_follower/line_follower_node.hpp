#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ai_msgs/msg/perception_targets.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "line_follower/steering_pd_controller.hpp"

namespace line_follower
{

enum class TrackState
{
  LOST = 0,
  TRACKING,
  YELLOW_LINE_DETECTED
};

struct ProcessingResult
{
  bool line_detected{false};
  bool yellow_line_detected{false};

  cv::Rect bounding_box{};
  cv::Point2f centroid{0.0F, 0.0F};

  double error{0.0};
  TrackState state{TrackState::LOST};

  cv::Mat roi_bgr;
  cv::Mat binary_mask;
  cv::Mat roi_hsv;

  int roi_top{0};
  int roi_bottom{0};
};

class LineFollowerNode : public rclcpp::Node
{
public:
  LineFollowerNode();

private:
  void declare_parameters();
  void setup_interfaces();

  void start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void digit_detection_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
  void reverse_brake_timer_callback();
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  void apply_digit_mask_to_roi(cv::Mat & mask, int roi_top) const;

  std::optional<ProcessingResult> process_image(const cv::Mat & frame_bgr) const;

  bool detect_red(
    const cv::Mat & frame_bgr,
    int roi_top,
    int roi_bottom,
    const cv::Point2d & line_centroid_roi,
    double & side_sign,
    cv::Point2d & red_centroid_img,
    cv::Point2d & div_base_img,
    cv::Point2d & div_target_img,
    const cv::Mat & white_mask = cv::Mat(),
    const cv::Mat & precomputed_hsv = cv::Mat()) const;

  std::string state_to_string(TrackState state) const;

  void publish_cmd_vel(
    double linear_speed,
    double yaw_rate,
    bool publish_stop);

  void publish_state(TrackState state);
  void publish_centroid(double norm_x, double norm_y);

  void publish_debug_image(
    const sensor_msgs::msg::Image::ConstSharedPtr & source,
    const cv::Mat & frame_bgr,
    const ProcessingResult & result) const;

  static double clamp(double value, double min_value, double max_value);

  // ============================================================
  // Topics
  // ============================================================
  std::string image_topic_;
  std::string cmd_vel_topic_;
  std::string debug_image_topic_;
  std::string line_state_topic_;
  std::string start_topic_;
  std::string stop_cmd_topic_;
  std::string line_centroid_topic_;

  // ============================================================
  // Basic motion
  // ============================================================
  double target_speed_{0.6};
  double max_speed_{1.0};
  double min_speed_{0.0};
  double max_yaw_rate_{1.2};
  double kp_{1.8};
  // 实验功能（默认关闭，kd=0.0）：可选低通滤波 PD 转向阻尼。
  double kd_{0.0};
  double kd_lpf_alpha_{0.35};
  double kd_max_contribution_{0.45};
  SteeringPdController steering_pd_;

  // ============================================================
  // ROI / image processing
  // ============================================================
  double roi_top_ratio_{0.5};
  double roi_bottom_ratio_{0.05};

  int gaussian_kernel_{5};
  double binary_threshold_{160.0};
  int morph_kernel_{5};
  int morph_iterations_{2};
  double min_contour_area_{2000.0};
  bool invert_binary_{false};

  cv::Mat morph_element_;

  // ============================================================
  // White road filter
  // ============================================================
  bool use_white_filter_{false};
  int white_h_low_{0};
  int white_h_high_{180};
  int white_s_max_{60};
  int white_v_min_{180};

  // ============================================================
  // Yellow line
  // ============================================================
  bool use_yellow_filter_{false};
  int yellow_h_low_{20};
  int yellow_h_high_{40};
  int yellow_s_min_{80};
  int yellow_v_min_{120};
  double yellow_area_ratio_threshold_{0.1};
  double yellow_stop_duration_{1.0};
  int yellow_permanent_count_{5};

  // 可选反向辅助制动；默认关闭。反向速度使用负值，脉冲由新鲜 Odom 截止。
  bool reverse_brake_enabled_{false};
  std::string odom_topic_{"/odom"};
  double reverse_speed_{0.0};
  double reverse_max_duration_{0.0};
  double reverse_cutoff_speed_{0.10};
  double reverse_abort_speed_{0.01};
  double reverse_odom_timeout_{0.15};
  bool reverse_brake_active_{false};
  bool odom_received_{false};
  double odom_linear_speed_{0.0};
  double odom_angular_speed_{0.0};
  rclcpp::Time odom_receive_time_;
  rclcpp::Time reverse_brake_start_time_;

  // 新增：同一条实体黄线只计数一次
  bool yellow_line_latched_{false};
  int yellow_clear_frames_{0};
  int yellow_clear_frames_required_{10};

  bool yellow_line_stop_active_{false};
  int yellow_line_detection_count_{0};
  bool permanent_stop_{false};
  rclcpp::Time yellow_line_detection_time_;

  // 实验功能（默认关闭）：黄线几何过滤与 /odom 停稳闭环确认。
  bool enable_yellow_geo_filter_{false};
  double yellow_min_width_ratio_{0.25};
  double yellow_min_aspect_ratio_{2.5};
  bool enable_yellow_odom_stop_confirm_{false};
  double yellow_still_speed_mps_{0.03};
  double yellow_still_hold_sec_{0.15};
  double yellow_stop_max_duration_{1.5};
  bool yellow_still_started_{false};
  rclcpp::Time yellow_still_start_time_;

  // ============================================================
  // Intersection probe
  // ============================================================
  bool enable_intersection_probe_{false};
  int intersection_band_height_{50};
  double intersection_dark_ratio_threshold_{0.9};
  double intersection_turn_speed_{0.2};
  double intersection_turn_yaw_rate_{0.8};
  double intersection_turn_duration_{1.0};
  double intersection_cooldown_{2.0};

  bool intersection_active_{false};
  rclcpp::Time intersection_end_time_;
  rclcpp::Time intersection_cooldown_until_;

  // ============================================================
  // Red obstacle avoidance
  // ============================================================
  bool enable_red_filter_{false};
  int red_h_low_1_{0};
  int red_h_high_1_{10};
  int red_h_low_2_{160};
  int red_h_high_2_{180};
  int red_s_min_{100};
  int red_v_min_{80};
  double red_area_ratio_threshold_{0.005};
  double red_turn_yaw_{0.6};
  double red_turn_linear_{0.0};
  double red_turn_duration_{1.0};
  int red_vote_frames_{8};
  double red_divider_angle_deg_{0.0};
  double red_divider_angle_rad_{0.0};

  // 实验功能（默认关闭）：红线/红禁行区优先按左右剩余白道宽度选边绕行。
  bool enable_red_white_width_side_{false};
  double red_white_side_margin_px_{10.0};

  // 软避障 / 红线分类 / 禁行区参数缓存（构造时一次读取，避免每帧 get_parameter）
  bool enable_soft_red_avoidance_{false};
  double red_soft_yaw_{0.30};
  double red_near_ratio_{0.35};
  double red_very_close_ratio_{0.72};
  double red_corridor_half_width_ratio_{0.42};
  double red_soft_speed_{0.15};
  double red_soft_curve_yaw_limit_{0.15};
  double red_line_aspect_ratio_min_{3.0};
  double red_line_min_width_ratio_{0.30};
  double red_cone_area_ratio_threshold_{0.0006};
  int red_cone_min_bbox_width_{4};
  int red_cone_min_bbox_height_{4};
  double red_repulsion_gain_{1.5};
  double red_lane_keep_min_factor_{0.40};
  double red_edge_guard_start_error_{0.15};
  double red_edge_guard_full_error_{0.30};
  double red_min_forward_speed_{0.06};
  int red_line_fallback_direction_{-1};
  double red_line_soft_yaw_{0.35};
  double red_line_speed_{0.12};

  bool enable_red_zone_lock_{false};
  int red_zone_close_kernel_w_{21};
  int red_zone_close_kernel_h_{21};
  double red_zone_min_width_ratio_{0.16};
  double red_zone_aspect_ratio_min_{0.5};
  double red_zone_aspect_ratio_max_{4.5};
  int red_zone_lock_clear_frames_{6};

  int red_left_votes_{0};
  int red_right_votes_{0};
  bool red_turning_{false};
  double red_turn_yaw_cmd_{0.0};
  rclcpp::Time red_turn_end_time_;

  bool last_red_overlay_valid_{false};
  cv::Point2d last_red_center_img_{0.0, 0.0};
  cv::Point2d last_div_base_img_{0.0, 0.0};
  cv::Point2d last_div_target_img_{0.0, 0.0};

  // ============================================================
  // Line state / runtime state
  // ============================================================
  double line_lost_stop_time_{0.8};
  double line_lost_forward_speed_{0.06};
  double line_lost_yaw_hold_factor_{0.85};
  double speed_gain_damping_{0.5};
  bool publish_debug_image_{true};
  std::string debug_image_mode_{"always"};
  bool reuse_process_hsv_for_red_{false};

  bool enable_continuous_curve_assist_{false};
  double reverse_curve_min_error_{0.12};
  double reverse_curve_threshold_{0.35};
  int reverse_curve_confirm_frames_{3};
  int reverse_curve_clear_frames_{4};
  double reverse_curve_speed_{0.16};
  double sharp_curve_threshold_{0.50};
  double sharp_curve_speed_{0.20};

  // Only used by the opt-in fork-map curve-preview experiment.
  bool enable_curve_preview_steering_{false};
  double curve_preview_max_far_weight_{0.45};
  double curve_preview_yaw_slew_rate_{8.0};
  bool curve_preview_yaw_initialized_{false};
  double curve_preview_last_yaw_rate_{0.0};
  rclcpp::Time curve_preview_last_time_;

  bool start_enabled_{false};
  TrackState last_state_{TrackState::LOST};
  rclcpp::Time last_detection_time_;

  // ============================================================
  // YOLO digit masking (opt-in)
  // ============================================================
  bool enable_yolo_digit_masking_{false};
  bool digit_mask_apply_to_yellow_{false};
  std::string digit_detection_topic_{"/hobot_dnn_detection"};
  double digit_mask_timeout_{0.10};
  double digit_mask_min_conf_{0.40};
  int digit_mask_pad_px_{8};
  bool has_digit_boxes_{false};
  std::vector<cv::Rect> latest_digit_boxes_;
  std::chrono::steady_clock::time_point latest_digit_steady_time_;

  // ============================================================
  // ROS interfaces
  // ============================================================
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_pub_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr digit_det_sub_;
  rclcpp::TimerBase::SharedPtr reverse_brake_timer_;

  image_transport::Publisher debug_pub_;
  bool debug_enabled_{false};
};

}  // namespace line_follower
