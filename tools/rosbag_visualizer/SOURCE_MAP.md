# C++ 与离线处理对应表

基准为本工作区 `line_follower_node.cpp`、`line_follower_node.hpp`、`config/*.yaml` 和 `src/camera/camera.py`。`line_follower.launch.py` 将 YAML 传给 `line_follower_node`。缺省值取 C++ 的参数声明，不从某一套比赛 YAML 猜测。

| 画面或输出 | C++ 执行位置 | 参数和复现口径 |
|---|---|---|
| 原图、时间戳 | 相机回调、`image_callback` | 相机发布 BGR8，header 在读帧后生成。离线使用图像 header，零戳或明显跨时钟域时回退到 bag 记录时间。 |
| ROI | `process_image` §1 | 顶部 `int(h*(1-roi_top_ratio))`，底部 `h-int(h*roi_bottom_ratio)`，再夹紧，底部不包含。 |
| 高斯、HSV | §2 | `gaussian_kernel` 至少为 1，偶数加 1，sigma=0。C++ Mat ROI 的边界可读父图像，Python 使用带边缘的裁切和 OpenCV 同款可分离高斯核复现子矩阵路径。HSV 格只做显示编码。 |
| 白色道路 Mask | §4 | `use_white_filter` 为真使用 `white_h_low/high`、`white_s_max`、`white_v_min`；否则使用 `binary_threshold`、`invert_binary`，两者互斥。 |
| 开与闭 | §5 | 矩形 `morph_kernel`，开 1 次，闭 `morph_iterations`，最少 1。 |
| 黄线 Mask | §3 | 模糊 ROI 的 HSV，`use_yellow_filter`、`yellow_h_low/high`、`yellow_s_min`、`yellow_v_min`、`yellow_area_ratio_threshold`（包含等于）。没有形态学处理。 |
| 红色 Mask 与几何 | `detect_red` 和 `image_callback` | 已启动、有道路目标且 `enable_red_filter` 时走独立未模糊 ROI；两个 `red_h_low/high_*` 区间取并集，`red_s_min/v_min`，固定 3×3 开闭各一次。按 `red_line_aspect_ratio_min`、`red_line_min_width_ratio`、`red_cone_area_ratio_threshold`、`red_cone_min_bbox_*` 分类，红锥优先。红线用左右余量和 `red_line_fallback_direction`，红锥用 `red_divider_angle_deg` 旋转后的分界线叉积。调试风险复算使用 `enable_soft_red_avoidance`、`red_vote_frames`、`red_near_ratio`、`red_very_close_ratio`、`red_corridor_half_width_ratio`。 |
| 白路轮廓 | §6 | RETR_EXTERNAL / CHAIN_APPROX_SIMPLE；`min_contour_area` 只筛最大轮廓质心兜底和框，局部带成功仍可跟踪。 |
| 采样带、目标 | §7–10 | 近 62–88%，远 34–56%。白列至少 `max(1,int(带高×0.30))`，连续宽至少 20 px。参考点落在某段内则距离为 0，同距离选宽段；远带优先参考近点。默认 70/30，按 `enable_continuous_curve_assist`、`reverse_curve_*` 确认和退出，预瞄用 `enable_curve_preview_steering`、`curve_preview_max_far_weight`。 |
| 调试图 | `publish_debug_image` | ROI 和闭运算 Mask 做 0.6/0.4 叠加，绘制框、目标、分界线、红色几何、黄线提示。标记重建。按续帧复算红色投票、走廊与风险。暂停或未录制启停时不沿用此前红色框；最终控制、计时器回调和 mux 直接看录制话题。 |
| 黄线事件 | `start_callback`、`image_callback` | 视觉检测和锁存计数分开。用 `yellow_clear_frames`、`yellow_permanent_count` 和已录制启停重建；控制日志优先用于确认实际触发，时长与里程使用录制指令和 Odom。 |

实现顺序为视觉帧处理与参数测试、Bag 读取和有界同步、注释/视频与制动指标、合成数据贯穿测试。ROS 控制与回调时序用录制话题观察，视觉按源帧逐张计算，编码再降采样。图像异常和断点在报告中计数，状态重建失去连续性时重置。速度曲线和事件计算使用独立的磁盘索引，内存随单帧和固定图表尺寸增长。
