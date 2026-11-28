# Handoff（2026-11-28 22:20）

## 1. 当前基线状态

- **仓库主分支**：`origin/main`（最新功能提交 `dbe9784`，单套双图统一工作区 `src/line_follow_ws/src/line_follower/`；只读备份 `origin/baseline/dual-20261126` 保持 `6a1303d` 勿动）
- **本次已验证入库改动（`dbe9784`）**：
  1. 环岛专属（`!passable_gap_enabled`）弯道红锥 `left_bounded` / `right_bounded` FOV 边界保护与 `stale_dead_end_latch` 释放，消除贴边裁切导致的 `one_side_dead_end` 反方向翻转撞锥；
  2. 可选开关 `enable_visual_finish_stop`（默认 `false`）两层视觉终点永停（Layer 1 黄线后水平收口黑条 + Layer 2 蓝色 `END` 垫白字连通域与横跳变，黄线停顿期 + 过黄线里程/时间双阶段门控），决赛不再依赖固定黄线次数；
  3. 可选开关 `use_image_center_ref`（默认 `false`），数字接管 `/cmd_vel_detection` 后以画面中心 `0.50` 作为参考；
  4. 新增环岛验证预设 [`YOLO_RING_STEP2_YAW165.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_STEP2_YAW165.yaml)（保持实车验证的 `yellow_stop_duration: 1.0`；`YOLO_RING_BASE.yaml` 与 `YOLO_FORK_BASE.yaml` 均未改动）。
- **主力 YOLO 模型**：[`yolo/v3_yolov8n/best_modified_yolov8_v3.bin`](yolo/v3_yolov8n/best_modified_yolov8_v3.bin)（`3.5 MB`, `md5:3cec559b`，环岛实测圈速与旋转数字表现略优于 `v5`）；备选 [`yolo/v5_yolov8s/best_modified_yolov8s_v5.bin`](yolo/v5_yolov8s/best_modified_yolov8s_v5.bin)（`11 MB`, `md5:bd98b8b5`）
- **校验与车端同步提醒**：
  - RDK X5 实机 `python3 -m unittest discover -s tests -p "test_*.py"` **31/31 通过**；
  - 车端 `/home/sunrise/aic_car_v2_20261126` 里的 `YOLO_RING_STEP2_YAW165.yaml` 曾临时试改为 `yellow_stop_duration: 0.5`，下次开机上车前请先执行 `deploy_to_car.sh` 同步 `origin/main`（恢复为已验证的 `1.0`）；
  - 车端另留有独立的 11-26 17:21 岔路回退工作区 `/home/sunrise/aic_car_fork_1126_1721`（原版 `v2` 模型 `md5:b75634e6`，`yellow_stop_duration: 1.0`，已于 `Nov 28 19:50` 重新 Release 编译）。

---

## 2. 当前最优参数与实测成绩

| 地图 | 当前推荐 YAML | 实测表现 | 关键参数特征 |
| --- | --- | --- | --- |
| **环岛图最优（`ring` 进阶）** | [`YOLO_RING_STEP2_YAW165.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_STEP2_YAW165.yaml) | **`v3`：`29.14s`**（`9.665m`, `LOST=0`, 红锥 `3x / 3.05s / max\|w\|=0.870 / lat_a=0.295 / flips=0`, 0 撞锥, 黄线滑行 `6.69–11.77cm`, Y4 `4/99` 两层视觉准确永停）；**`v5`：`29.85s`**（`9.816m`, `LOST=0`, 红锥 `3x / 3.165s / max\|w\|=0.716 / lat_a=0.305 / flips=0`, Y4 `30.06s` 确认永停 `blk=0.86, end_B=0.91, blobs=3, tr=12`）；Y1–Y3 均零误停（较修复前 `31.79s/30.90s, flips=2` 撞锥明显提升） | `target_speed: 0.52`, `red_avoid_max_yaw_rate: 1.65`, `use_image_center_ref: true`, `cross_below_margin: 0.06`, `yellow_stop_duration: 1.0`, `yellow_permanent_count: 99`, `enable_visual_finish_stop: true` |
| **环岛图基线（`ring`）** | [`YOLO_RING_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_BASE.yaml) | **`0.52 m/s`，3/3 圈 `30.43–31.96s`**，`4/4` 数字全过，黄线 odom 反向制动 `8.6–11.7 cm` | `target_speed: 0.52`, `kp: 2.85`, `kd: 0.20`, `roi_top_ratio: 0.58`, `red_roi_top_ratio: 0.45`, `red_avoid_max_yaw_rate: 1.25`, `enable_reverse_brake: true`, `enable_tjunction_merge: false`, `enable_passable_gap_red_avoid: false` |
| **环岛进阶提速（待对比）** | [`YOLO_RING_OPT4_S060_LATCAP.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/ring_opt4_20261127/YOLO_RING_OPT4_S060_LATCAP.yaml) / [`OPT3_S060`](src/line_follow_ws/src/line_follower/config/parameter_presets/ring_opt4_20261127/YOLO_RING_OPT3_S060_PREVIEW_RB.yaml) | `OPT3_S060` 曾跑出 **`29.16s`**；`OPT4` 为 `0.60 m/s` 侧向加速度限幅版（待上车实测对比） | `red_avoid_max_lat_accel: 0.34`, `red_avoid_highspeed_slew_ref_speed: 0.30` |
| **环岛稳健热身** | [`YOLO_RING_SAFE_S028.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_SAFE_S028.yaml) | `0.28 m/s` 稳健版 | 新场地光线变化或首次热车用 |
| **岔路图 (`fork`)** | [`YOLO_FORK_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_FORK_BASE.yaml) | **`0.28 m/s`，`FORK_GAP_FIX1`**：`5 -> 8 -> ┤ T并线(0.78m, 167行, 1.30s) -> 镂空红区 -> 1号红锥 -> 黄线3` 全过（`LOST=0`） | `roi_top_ratio: 0.65`, `min_winner_conf: 0.55`, `enable_tjunction_merge: true`, `enable_passable_gap_red_avoid: true`, `tjunction_min_arm_dist: 0.75`, `tjunction_min_full_rows: 80`, `tjunction_min_duration: 1.30` |

---

## 3. 一键启动方式（复制到车端即可运行）

**1) 环岛图（当前最快已验证方案，含弯道防撞锥 + 两层视觉终点永停，`29.14s`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && \
bash src/scripts/start_line_following.sh \
  --map ring \
  --params YOLO_RING_STEP2_YAW165.yaml \
  --yolo --yolo-model best_modified_yolov8_v3.bin \
  --record-tag RING_2LAYER_END
```

**2) 环岛图（保守稳定基线 `YOLO_RING_BASE.yaml`，`0.52 m/s`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && \
bash src/scripts/start_line_following.sh \
  --map ring \
  --params YOLO_RING_BASE.yaml \
  --yolo --yolo-model best_modified_yolov8_v3.bin \
  --record-tag RING_BASE
```

**3) 岔路图（Fork Map，当前最佳验证基线 `FORK_GAP_FIX1`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && \
bash src/scripts/start_line_following.sh \
  --map fork \
  --params YOLO_FORK_BASE.yaml \
  --yolo --yolo-model best_modified_yolov8_v3.bin \
  --record-tag FORK_BASE
```

**4) 交互菜单模式（直接回车即选对应地图 `#1 [默认推荐基线]`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && bash src/scripts/start_line_following.sh
```
> 按 **空格** 启停车辆；跑完按 **Ctrl+C** 自动刹停并等待 rosbag 索引落盘。

---

## 4. 未解决卡点、已证伪思路与下一步建议

1. **环岛第 2 组 & 第 4 组旋转数字漏检 / 误判（已定位根因，暂未改代码）**：
   - **第 4 组（左 `3`、右 `8`，约 90° 侧躺视角）**：实测 `v3` 与 `v5` 都会将侧躺 `3` 高置信度误判为 `8 (0.90–0.93)`、将侧躺 `8` 误判为 `5/4 (0.89–0.94)`，属于训练集缺少实车地面 90° 旋转视角样本的模型瓶颈；建议从车端实录图补标 90° 旋转数字样本并微调模型。
   - **第 2 组**：`detection_turn_controller.cpp` 中 `!any_border_urgent`（`border-early`）会让左边缘先入框的单个数字跳过 `dual_track_hold_time` 提前以 `tracks:1` 触发决策，右侧数字（`8`）尚未完全入框；且 `v5` 易把旋转 `8` 误判为 `4/5`。后续可针对 `border-early` 触发门限单独做一次小步实测。
2. **岔路图 2 号红锥（右发卡弯出口，`FORK_GAP_FIX1` 包中 `dt=36.5s–39.2s`）**：
   - 车刚贴近左白道进入 2 号红锥时，锥左缘贴到画面左边界（`x=0..89`，`wL=0`），触发右绕把车推出黑线。
   - **已证伪思路**：`FORK_GAP_FIX2`（`hold_clipped_cone` / `edge_clipped`）会导致 1 号锥也过不去，已移除并在 `tests/test_unified_node_consistency.py` 中写死回归保护（**严禁重新引入同名变量/逻辑**）；排查时请重点用 `analyze_bag.py` 看 2 号锥入框时的偏航角、`red_near_ratio` 与左边界余量。
3. **未验证参数与提速路线**：
   - **`yellow_stop_duration: 0.5`** 尚未完成录包验证，按阶段 3 原则未入库（`main` 保持 `1.0`）；后续如需压缩黄线停顿，请实跑录包确认 `stage1-stop` 两层视觉终点检测（需连续 3 帧命中）和滑行距离不回退。
   - **模型对照结论**：`v3` 与 `v5` 在环岛新逻辑下均已实测 0 撞锥、0 丢线、两层终点准确永停，但 `v3` 圈速更快（`29.14s` vs `29.85s`）且旋转数字误判更少，当前继续以 `best_modified_yolov8_v3.bin` 为主力；后续可在稳态基础上继续推进 `OPT4`（环岛 `0.60 m/s`）与 `EXP1..EXP4`（岔路）梯度提速。
