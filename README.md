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
- `src/line_follow_ws`：C++ 巡线避障、YOLO 数字转向与 4 级速度多路复用工作区
  - `include/line_follower/red_lat_accel_limiter.hpp`：OPT4 高速避障侧向加速度 & 自适应转向变化率限幅器
  - `config/parameter_presets/ring_opt4_20261127/`：11-26 晚优胜基线（`OPT3_S044` / `OPT3_S052`）及 11-27 实车 `OPT4` 6 组实验预设
- `src/camera`、`src/yolov8`：相机与 BPU YOLOv8 数字检测节点
- `src/scripts`：一键启动（`start_line_following.sh`）、OPT4 部署（`apply_opt4_to_v2.sh`）、车端同步（`sync_v2_from_car.sh`）、计时器（`lap_timer.py`）
- `tests`：不依赖 ROS 运行时的纯 C++17 / Python 单元测试
- `tools/rosbag_visualizer`：ROS 2 录包离线分析与视频面板渲染工具
- `docs`：赛前实车记录、实验方案（含 [`docs/实车测试/20261127_高速防摆动OPT4实验方案.md`](docs/实车测试/20261127_高速防摆动OPT4实验方案.md)）与历史参数归档（`docs/archive/`）

## 快速开始

### 1. 本机运行离线单元测试
```bash
clang++ -std=c++17 -I src/line_follow_ws/src/line_follower/include tests/test_red_lat_accel_limiter.cpp -o /tmp/test_lat && /tmp/test_lat && rm -f /tmp/test_lat
python3 -m unittest discover -s tests -p "test_*.py"
```

### 2. 部署 OPT4 高速防摆模块到车端 `aic_car_v2_20261126`
```bash
bash src/scripts/apply_opt4_to_v2.sh sunrise@172.20.10.3 /home/sunrise/aic_car_v2_20261126
```

### 3. 车端启动巡线（以 OPT4 E2 为例）
```bash
cd /home/sunrise/aic_car_v2_20261126 && \
bash src/scripts/start_line_following.sh --map ring --params YOLO_RING_OPT4_S060_LATCAP.yaml --yolo
```

### 4. 从车端回传同步源码到本地快照
```bash
bash src/scripts/sync_v2_from_car.sh sunrise@172.20.10.3 /home/sunrise/aic_car_v2_20261126
```
