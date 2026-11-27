# Handoff（2026-11-28）

## 1. 当前基线状态

- **仓库主分支**：`origin/main`（已统合双图单一工作区 `src/line_follow_ws/src/line_follower/`；只读备份 `origin/baseline/dual-20261126` 保持 `6a1303d` 勿动）
- **主力 YOLO 模型**：[`yolo/v3_yolov8n/best_modified_yolov8_v3.bin`](yolo/v3_yolov8n/best_modified_yolov8_v3.bin)（`3.5 MB`, `md5:3cec559b`）；备选新版 [`yolo/v5_yolov8s/best_modified_yolov8s_v5.bin`](yolo/v5_yolov8s/best_modified_yolov8s_v5.bin)（`11 MB`, `md5:bd98b8b5`）
- **上车前验证状态**：Mac 离线 `unittest`（31 项）+ C++17 单测（7 组 43 项）+ `bash -n` 全部通过；上车后请先跑一次 `deploy_to_car.sh`（或车端 `build_on_car.sh`）完成 `Release` 编译

---

## 2. 当前最优参数与实测成绩

| 地图 | 当前最优 YAML | 实测表现 | 关键参数特征 |
| --- | --- | --- | --- |
| **环岛图 (`ring`)** | [`YOLO_RING_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_BASE.yaml) | **`0.52 m/s`，3/3 圈 `30.43–31.96s`**，`4/4` 数字全过，黄线 odom 反向制动 `8.6–11.7 cm` | `target_speed: 0.52`, `kp: 2.85`, `kd: 0.20`, `roi_top_ratio: 0.58`, `red_roi_top_ratio: 0.45`, `red_avoid_max_yaw_rate: 1.25`, `enable_reverse_brake: true`, `enable_tjunction_merge: false`, `enable_passable_gap_red_avoid: false` |
| **环岛进阶提速** | [`YOLO_RING_OPT4_S060_LATCAP.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/ring_opt4_20261127/YOLO_RING_OPT4_S060_LATCAP.yaml) / [`OPT3_S060`](src/line_follow_ws/src/line_follower/config/parameter_presets/ring_opt4_20261127/YOLO_RING_OPT3_S060_PREVIEW_RB.yaml) | `OPT3_S060` 曾跑出 **`29.16s`**；`OPT4` 为 `0.60 m/s` 侧向加速度限幅版（待上车实测对比） | `red_avoid_max_lat_accel: 0.34`, `red_avoid_highspeed_slew_ref_speed: 0.30` |
| **环岛稳健热身** | [`YOLO_RING_SAFE_S028.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_SAFE_S028.yaml) | `0.28 m/s` 稳健版 | 新场地光线变化或首次热车用 |
| **岔路图 (`fork`)** | [`YOLO_FORK_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_FORK_BASE.yaml) | **`0.28 m/s`，`FORK_GAP_FIX1`**：`5 -> 8 -> ┤ T并线(0.78m, 167行, 1.30s) -> 镂空红区 -> 1号红锥 -> 黄线3` 全过（`LOST=0`） | `roi_top_ratio: 0.65`, `min_winner_conf: 0.55`, `enable_tjunction_merge: true`, `enable_passable_gap_red_avoid: true`, `tjunction_min_arm_dist: 0.75`, `tjunction_min_full_rows: 80`, `tjunction_min_duration: 1.30` |

---

## 3. 一键启动方式（复制到车端即可运行）

**1) 环岛图（Ring Map，当前最优稳定基线 0.52 m/s）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && \
bash src/scripts/start_line_following.sh \
  --map ring \
  --params YOLO_RING_BASE.yaml \
  --yolo --yolo-model best_modified_yolov8_v3.bin \
  --record-tag RING_BASE
```

**2) 岔路图（Fork Map，当前最佳验证基线 `FORK_GAP_FIX1`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && \
bash src/scripts/start_line_following.sh \
  --map fork \
  --params YOLO_FORK_BASE.yaml \
  --yolo --yolo-model best_modified_yolov8_v3.bin \
  --record-tag FORK_BASE
```

**3) 交互菜单模式（直接回车即选对应地图 `#1 [默认推荐基线]`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && bash src/scripts/start_line_following.sh
```
> 按 **空格** 启停车辆；跑完按 **Ctrl+C** 自动刹停并等待 rosbag 索引落盘。

---

## 4. 下一步重点待调项

1. **岔路图 2 号红锥（右发卡弯出口，`FORK_GAP_FIX1` 包中 `dt=36.5s–39.2s`）**：
   - 车刚贴近左白道进入 2 号红锥时，锥左缘贴到画面左边界（`x=0..89`，`wL=0`），触发右绕把车推出黑线。注意此前 `FORK_GAP_FIX2`（`hold_clipped_cone`）会导致 1 号锥也过不去，已移除；排查时请重点用 `analyze_bag.py` 看 2 号锥入框时的偏航角、`red_near_ratio` 与左边界余量。
2. **模型与高速对照**：
   - 在 `YOLO_RING_BASE.yaml` 与 `YOLO_FORK_BASE.yaml` 各自跑稳后，可录包对比 `best_modified_yolov8s_v5.bin` 与 `v3.bin`；确认稳定后再按 `OPT4`（环岛）和 `EXP1..EXP4`（岔路）梯度提速。
