# AIC-car-final

全球校园人工智能算法精英大赛（AIC）智能车视觉规划赛代码库（RDK X5 + ROS 2 Humble / TROS + Wheeltec 阿克曼底盘）。

**当前 `main` 已统一为单套双图基线**：同一份 C++ 节点通过 YAML 预设无缝切换**环岛图（Ring）**与**岔路图（Fork）**，不再存在两套分裂工作区。

## 仓库分支说明

| 分支 | 用途与说明 |
| --- | --- |
| **`main`** | **团队唯一开发与部署主分支**：统一版 C++ 巡线/避障/T-merge/数字转向/4 级 MUX 节点，合并昨晚已验证环岛优胜表现（`0.52 m/s` 3 圈稳跑 `30.43–31.96s`）与今晚最佳岔路表现（`FORK_GAP_FIX1`），配齐一键部署、机载编译、录包分析与离线单测。 |
| **`baseline/dual-20261126`** | 早期车端双工作区快照分支，**保持只读存档，请勿修改或推送**。 |

---

## 核心能力与默认推荐基线

| 地图 | 默认推荐 YAML | 验证速度与成绩 | 核心机制（通过 YAML 模式开关零干扰隔离） |
| --- | --- | --- | --- |
| **环岛图 (`ring`)** | [`YOLO_RING_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_BASE.yaml) | `0.52 m/s`，实测 3/3 圈 `30.43–31.96s`，4/4 数字，odom 反向制动 `8.6–11.7cm` | `enable_tjunction_merge: false`, `enable_passable_gap_red_avoid: false`；开启 `OPT3` 上沿红障预瞄、反鞭限幅、`one_side_dead_end` 外锥保护；高阶速度档 [`ring_opt4_20261127/`](src/line_follow_ws/src/line_follower/config/parameter_presets/ring_opt4_20261127/) 支持 `OPT4` 侧向加速度限幅（`0.56–0.64 m/s`） |
| **岔路图 (`fork`)** | [`YOLO_FORK_BASE.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_FORK_BASE.yaml) | `0.28 m/s`，`FORK_GAP_FIX1` 验证：`5 -> 8 -> ┤并线(0.78m) -> 红区 -> 锥1 -> 黄线3` 全通（`LOST=0`） | `enable_tjunction_merge: true`, `enable_passable_gap_red_avoid: true`；`0.65` 近地 ROI + 16:28 多目标轨迹投票（`min_winner_conf: 0.55`）+ `┤` 丁字全白行并线 + 未桥接 `binary_mask` 可通行缺口制导；进阶档见 [`fork_teammate_exp_20261127/`](src/line_follow_ws/src/line_follower/config/parameter_presets/fork_teammate_exp_20261127/) |
| **环岛热身备份** | [`YOLO_RING_SAFE_S028.yaml`](src/line_follow_ws/src/line_follower/config/parameter_presets/YOLO_RING_SAFE_S028.yaml) | `0.28 m/s` 稳健基础版 | 赛场光线骤变或刚换场地时用于快速摸底 |

> 说明：更多实车细节、岔路地图 2 号红锥待调点与团队交接说明见 [`20261128_明日队友交接与双图统一基线指南.md`](docs/实车测试/20261128_明日队友交接与双图统一基线指南.md)。

---

## 明日团队成员上车工作流

### 第一步：拉取仓库并部署到小车

无论哪位同学带小车测试，请统一从 `main` 拉取最新代码：

```bash
git clone git@github.com:SilverLi123/AIC-car-final.git   # 若已克隆则：git pull --rebase origin main
cd AIC-car-final
```

**方式 A（推荐，队友笔记本 -> 一键 SSH 同步并编译到小车，支持 macOS / Linux / Windows Git Bash & WSL）：**
```bash
bash src/scripts/deploy_to_car.sh sunrise@<小车IP> /home/sunrise/aic_car_v2_20261126
# 例如热连接 172.20.10.3 时直接：
bash src/scripts/deploy_to_car.sh
```
脚本会自动：备份车端原代码到 `backups/` -> 同步统一 C++ 包、全部预设、脚本与 YOLO `.bin` 模型 -> 清理冲突 `CMakeCache` -> 以 `Release` 模式编译 `line_follower` -> 打印环岛/岔路启动命令。

**方式 B（小车直连手机热点有网时，直接在 RDK X5 车端拉取并编译）：**
```bash
cd /home/sunrise/AIC_car_final   # 或车端对应 git 仓库目录
git pull --rebase origin main
bash src/scripts/build_on_car.sh
```

---

### 第二步：选择地图、模型并一键发车 + 录包

**1) 环岛图（Ring Map，默认推荐基线 0.52 m/s）：**
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

**3) 交互菜单模式（忘记文件名时直接运行，回车即选默认基线 `#1`）：**
```bash
cd /home/sunrise/aic_car_v2_20261126 && bash src/scripts/start_line_following.sh
```

> 操作提示：启动后在终端按 **空格** 启动/暂停，按 **Ctrl+C** 自动停车并等待录包索引完整落盘。

---

### 第三步：YOLO 模型与录包对比分析

可用模型（运行 `bash src/scripts/start_line_following.sh --list-models` 查看大小与 MD5）：
- `best_modified_yolov8_v3.bin`（`3.5 MB`, `md5:3cec559b`，位于 `yolo/v3_yolov8n/`）：昨晚环岛 4/4 数字与今晚岔路已验证主力模型
- `best_modified_yolov8s_v5.bin`（`11 MB`, `md5:bd98b8b5`，位于 `yolo/v5_yolov8s/`）：全量真图+合成重训 YOLOv8s（实测对照 `v3` 后可择优切换）
- `best_modified_yolov8.bin`（`11 MB`, `md5:8d6f45eb`，位于 `src/yolov8/mod1/`）：原赛题基线备份

录包分析：
```bash
# 对比同一批实验多个录包的耗时、速度、丢失率与黄线停靠情况
python3 src/scripts/analyze_bag.py --compare /mnt/aic_usb/aic_bags_YYYYMMDD/
```

---

## 团队 Git 协作约定（避免工作区再次分叉）

1. **禁止只改车端不提交**：所有在车上验证通过的 C++ 或 YAML 改动，回到电脑后立刻拉回本仓库提交到 GitHub，保持团队永远围绕同一套基线迭代。
2. **禁止修改或强推 `baseline/dual-20261126`**，该分支只作历史只读备份。
3. **调参必须新建或明确命名 `--record-tag`**，保证每个包都生成绑定模型与预设的 `run_meta.json`。
4. **每次提交前本机跑一遍离线单测与一致性校验**：
   ```bash
   python3 -m unittest discover -s tests -p "test_*.py" -v
   ```

---

## 仓库目录结构

- `src/car_ws`：Wheeltec 底盘 ROS 2 工作区
- `src/line_follow_ws/src/line_follower`：统一 C++ 巡线、避障、`┤` T-merge、多目标数字投票转向与 4 级 MUX 节点
  - `include/line_follower/passable_gap_guidance.hpp`：岔路未桥接白路可通行缺口制导
  - `include/line_follower/red_lat_accel_limiter.hpp`：OPT4 高速避障侧向加速度与转向变化率限幅
  - `config/parameter_presets/`：`YOLO_RING_BASE.yaml`、`YOLO_RING_SAFE_S028.yaml`、`YOLO_FORK_BASE.yaml`，以及进阶梯度 `ring_opt4_20261127/` 与 `fork_teammate_exp_20261127/`
- `src/camera`、`src/yolov8`、`yolo`：相机节点、BPU YOLOv8 推理节点与多版本 `.bin` 权重库
- `src/scripts`：`deploy_to_car.sh`、`build_on_car.sh`、`start_line_following.sh`、`record_bag.sh`、`analyze_bag.py`
- `tests`：不依赖 ROS 运行时的纯 C++17 / Python 单元测试与 YAML-C++ 参数一致性检查
- `docs`：赛前实车记录与档案（废弃的早期单工作区脚本/预设已整理至 `docs/archive/`）
