# AIC-car-final

全球校园人工智能算法精英大赛（AIC）智能车视觉规划赛代码库（RDK X5 + ROS 2 Humble / TROS + Wheeltec 阿克曼底盘）。

## 仓库分支说明

| 分支 | 用途与说明 |
| --- | --- |
| **`main`** | 主开发与实验整理分支：包含重构模块（PD 控制、红障侧向加速度防摆限幅、白路空间雕刻、YOLO 投票过滤）、现场测试预设与自动化部署/回传脚本、离线单元测试与 rosbag 可视化工具。 |
| **`baseline/dual-20261126`** | 车端双地图（`ws_aic_dual_20261124`：环岛 + 岔路独立工作区）稳定基线快照分支，保持只读存档，便于随时回退对比。 |

> 说明：当前实车主力目录为车端 `/home/sunrise/aic_car_v2_20261126`（环岛 4/4 数字 + 弯道预瞄 + 反向制动）。通过 `src/scripts/apply_opt4_to_v2.sh` 可一键把本仓库的高速防摆模块与 `OPT4` 实验预设部署到车端并编译；通过 `src/scripts/sync_v2_from_car.sh` 可随时把车端工程拉回本地。

## 目录结构

- `src/car_ws`：Wheeltec 底盘 ROS 2 工作区
- `src/line_follow_ws`：C++ 巡线避障、YOLO 数字转向与 4 级速度多路复用工作区（**单套统一二进制同时支持 Ring 环岛图与 Fork 岔路图**，仅通过 YAML 参数切换）
  - `include/line_follower/red_lat_accel_limiter.hpp`：OPT4 高速避障侧向加速度 & 自适应转向变化率限幅器
  - `include/line_follower/digit_decision_logic.hpp`：双目标配对延时保持、跨帧空间槽记忆（抗反光）与单帧高置信否决
  - `include/line_follower/tjunction_merge_detector.hpp`：抗斜切与抗红障遮挡的 `┤` 丁字并线 + 防丢线救护状态机
  - `config/parameter_presets/ring_opt4_20261127/`：环岛图 `OPT3` 优胜基线与 `OPT4` 实验预设（9 套：`R0`–`R8`）
  - `config/parameter_presets/fork_opt4_20261127/`：岔路图统一系统 `OPT4` 预设（6 套：`F1`–`F6`）
- `src/camera`、`src/yolov8`：相机与 BPU YOLOv8 数字检测节点（支持通过 `--yolo-model` 切换 `.bin` 模型）
- `src/scripts`：
  - `start_line_following.sh`：统一交互/命令行启动器（支持 `--map`、`--params`、`--yolo` / `--yolo-model`、`--record` 一键同步录包）
  - `record_bag.sh` + `analyze_bag.py`：zstd 压缩录包与 `run_meta.json` 绑定、单包与多包横向统计对比工具
  - `deploy_unified_to_car.sh` / `apply_opt4_to_v2.sh` / `sync_v2_from_car.sh` / `lap_timer.py`：车端一键同步部署、拉回快照与计时器
- `tests`：不依赖 ROS 运行时的纯 C++17 / Python 单元测试
- `tools/rosbag_visualizer`：ROS 2 录包离线分析与视频面板渲染工具
- `docs`：赛前实车记录与实验方案（含 [`docs/实车测试/20261127_今晚双图实车实验总表.md`](docs/实车测试/20261127_今晚双图实车实验总表.md)、[`docs/实车测试/20261127_高速防摆动OPT4实验方案.md`](docs/实车测试/20261127_高速防摆动OPT4实验方案.md) 与 [`docs/实车测试/20261127_Fork岔路与统一双图改造说明.md`](docs/实车测试/20261127_Fork岔路与统一双图改造说明.md)）与历史参数归档（`docs/archive/`）

## 快速开始

### 1. 本机运行离线单元测试
```bash
python3 -m unittest discover -s tests -p "test_*.py"
```

### 2. 一键同步统一系统 + 脚本 + 全部预设到车端 `aic_car_v2_20261126` 并编译
```bash
bash src/scripts/deploy_unified_to_car.sh sunrise@172.20.10.3 /home/sunrise/aic_car_v2_20261126
```

### 3. 车端启动巡线（交互选择地图 / 参数 / YOLO 模型 / 同步录包）
```bash
cd /home/sunrise/aic_car_v2_20261126 && bash src/scripts/start_line_following.sh
```
或单行指定参数 + 模型 + 录包（详见 [`20261127_今晚双图实车实验总表.md`](docs/实车测试/20261127_今晚双图实车实验总表.md)）：
```bash
cd /home/sunrise/aic_car_v2_20261126 && \
bash src/scripts/start_line_following.sh --map ring --params YOLO_RING_OPT4_S060_LATCAP.yaml --yolo --yolo-model best_modified_yolov8.bin --record-tag R3_S060
```

### 4. 多包横向分析与生成对比表
```bash
python3 src/scripts/analyze_bag.py --compare recorded_bags/ --markdown docs/实车测试/20261127_今晚录包汇总.md
```

### 5. 从车端回传同步源码到本地快照
```bash
bash src/scripts/sync_v2_from_car.sh sunrise@172.20.10.3 /home/sunrise/aic_car_v2_20261126
```
