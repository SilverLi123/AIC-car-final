# AIC-car-final

全球校园人工智能算法精英大赛（AIC）智能车视觉规划赛代码库（RDK X5 + ROS 2 Humble / TROS + Wheeltec 阿克曼底盘）。

**当前 `main` 已统一为单套双图基线**：同一份 C++ 节点通过 YAML 预设无缝切换**环岛图（Ring）**与**岔路图（Fork）**，不再存在两套分裂工作区。

## 仓库分支说明

| 分支 | 用途与说明 |
| --- | --- |
| **`main`** | **唯一开发与部署主分支**：统一版 C++ 巡线/避障/T-merge/数字转向/4 级 MUX 节点，合并环岛优胜表现（`0.52 m/s` 3 圈稳跑 `30.43–31.96s`）与岔路最佳表现（`FORK_GAP_FIX1`），配套一键部署、机载编译、录包分析与离线单测。 |
| **`baseline/dual-20261126`** | 早期车端双工作区快照分支，**保持只读存档，请勿修改或推送**。 |

---

## 开发与实车测试工作流

### 阶段 1：开始前检查本地与车端状态，并更新车端工作区

每次开始开发或上车测试前，**必须先确认 GitHub 仓库与小车本地工作区的文件差异**，防止覆盖车上尚未回传的改动：

1. **拉取仓库最新代码并阅读交接状态**：
   ```bash
   git pull --rebase origin main
   cat handoff.md
   ```
2. **检查小车本地与当前仓库的文件差异（dry-run，只看不改）**：
   ```bash
   rsync -avnc --delete \
     --exclude '*.bak*' --exclude '__pycache__/' --exclude '.DS_Store' \
     src/line_follow_ws/src/line_follower/ \
     sunrise@<小车IP>:/home/sunrise/aic_car_v2_20261126/src/line_follow_ws/src/line_follower/

   rsync -avnc --exclude '__pycache__/' --exclude '.DS_Store' \
     src/scripts/ \
     sunrise@<小车IP>:/home/sunrise/aic_car_v2_20261126/src/scripts/
   ```
   - 若发现小车上有之前遗留但还没入库的有用改动，先手动 diff 并决定保留或舍弃后再继续。
3. **将小车工作区更新到最新并编译**：
   - **电脑 SSH 一键同步 + Release 编译（推荐，支持 macOS / Linux / Windows Git Bash & WSL）**：
     ```bash
     bash src/scripts/deploy_to_car.sh sunrise@<小车IP> /home/sunrise/aic_car_v2_20261126
     # 默认连接 sunrise@172.20.10.3 时直接执行：
     bash src/scripts/deploy_to_car.sh
     ```
     脚本会自动在车端 `backups/` 生成时间戳备份，随后同步 `line_follower`、全部预设、脚本、`README.md`、`handoff.md` 与 YOLO `.bin` 模型，清理冲突 `CMakeCache` 并以 `Release` 模式编译。
   - **或直接在连接网络的小车端拉取并编译**：
     ```bash
     cd /home/sunrise/AIC_car_final
     git pull --rebase origin main
     bash src/scripts/build_on_car.sh
     ```

---

### 阶段 2：选图发车、同步录包与量化分析

**1) 环岛图（Ring Map，默认推荐基线 `0.52 m/s`）：**
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

**3) 交互菜单模式（按回车即选 `#1 [默认推荐基线]`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && bash src/scripts/start_line_following.sh
```
> 启动后在终端按 **空格** 启停；按 **Ctrl+C** 自动刹停并等待 rosbag 索引完整落盘。

**4) 录包量化对比与模型查看：**
```bash
# 查看所有自动发现的 YOLO .bin 模型及 MD5
bash src/scripts/start_line_following.sh --list-models

# 横向对比本轮多个录包的耗时、速度、丢线与红区/黄线指标
python3 src/scripts/analyze_bag.py --compare /mnt/aic_usb/aic_bags_YYYYMMDD/
```

---

### 阶段 3：小步勤提交，确认效果有提升再提交

1. **禁止未验证入库**：每次参数或 C++ 逻辑调整后，先上车实跑并录包（通过 `--record-tag` 命名），用实跑结果和 `analyze_bag.py` 数据确认效果**确实有提升且核心路段无回退**。
2. **效果变差立即舍弃/回退**：如果某次改动不如上一版（例如更易丢线、撞锥或触发延迟），直接回退该改动，不要把负优化提交到 `main`。
3. **有效改动小步勤提交**：只要有一项明确收益的改进验证通过，在本机跑通离线校验后立刻 `git commit` 并推送到 `origin/main`，避免大堆改动混在一起或再次出现本机/车端代码分裂：
   ```bash
   python3 -m unittest discover -s tests -p "test_*.py" -v
   git add -A && git commit -m "<清晰的改动说明与实测结论>"
   git push origin main
   ```

---

### 阶段 4：结束工作前更新 `handoff.md`

每次收工或交接前，务必更新仓库根目录的 [`handoff.md`](handoff.md) 并一并提交推送到 `origin/main`，内容必须写清：
1. **当前双图最优 YAML 预设、YOLO `.bin` 模型及实测指标**；
2. **环岛 / 岔路可直接复制运行的启动命令**；
3. **当前未解决的卡点、已证伪的思路及下一步建议**。

---

## 核心预设与机制一览

| 地图 | 默认推荐 YAML | 已验证表现 | 模式开关与核心机制 |
| --- | --- | --- | --- |
| **环岛图 (`ring`)** | [`YOLO_RING_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_BASE.yaml) | `0.52 m/s`，实测 3/3 圈 `30.43–31.96s`，4/4 数字，odom 反向制动 `8.6–11.7cm` | `enable_tjunction_merge: false`, `enable_passable_gap_red_avoid: false`；`OPT3` 上沿红障预瞄、反鞭限幅、`one_side_dead_end` 外锥保护；进阶提速见 [`ring_opt4_20261127/`](src/line_follow_ws/src/line_follower/config/parameter_presets/ring_opt4_20261127/) |
| **岔路图 (`fork`)** | [`YOLO_FORK_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_FORK_BASE.yaml) | `0.28 m/s`，`FORK_GAP_FIX1` 验证：`5 -> 8 -> ┤并线(0.78m) -> 红区 -> 锥1 -> 黄线3`（`LOST=0`） | `enable_tjunction_merge: true`, `enable_passable_gap_red_avoid: true`；`0.65` 近地 ROI + 多目标投票（`min_winner_conf: 0.55`）+ `┤` 丁字全白行并线 + 未桥接 `binary_mask` 缺口制导；进阶档见 [`fork_teammate_exp_20261127/`](src/line_follow_ws/src/line_follower/config/parameter_presets/fork_teammate_exp_20261127/) |
| **环岛热身备份** | [`YOLO_RING_SAFE_S028.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_SAFE_S028.yaml) | `0.28 m/s` 稳健基础版 | 光线变化或新场地摸底用 |

> 详细调优细节与岔路 2 号红锥分析见 [`20261128_双图统一基线与实车调优指南.md`](docs/实车测试/20261128_双图统一基线与实车调优指南.md)。

## 仓库目录结构

- `handoff.md`：当前最优参数、启动命令与遗留问题交接清单（每次收工更新）
- `src/car_ws`：Wheeltec 底盘 ROS 2 工作区
- `src/line_follow_ws/src/line_follower`：统一 C++ 巡线、避障、`┤` T-merge、多目标数字投票转向与 4 级 MUX 节点
  - `include/line_follower/passable_gap_guidance.hpp`：岔路未桥接白路可通行缺口制导
  - `include/line_follower/red_lat_accel_limiter.hpp`：OPT4 高速避障侧向加速度与转向变化率限幅
  - `config/parameter_presets/`：`YOLO_RING_BASE.yaml`、`YOLO_RING_SAFE_S028.yaml`、`YOLO_FORK_BASE.yaml`，以及进阶梯度 `ring_opt4_20261127/` 与 `fork_teammate_exp_20261127/`
- `src/camera`、`src/yolov8`、`yolo`：相机节点、BPU YOLOv8 推理节点与 `.bin` 权重库（`v3_yolov8n` / `v5_yolov8s` / `mod1`）
- `src/scripts`：`deploy_to_car.sh`、`build_on_car.sh`、`start_line_following.sh`、`record_bag.sh`、`analyze_bag.py`
- `tests`：不依赖 ROS 运行时的纯 C++17 / Python 单元测试与 YAML-C++ 参数一致性校验
- `docs`：赛前实车记录与档案（早期分裂脚本/预设已归档至 `docs/archive/`）
