# 环岛图比赛三组参数备份（2026-10-31）

这三组是同一套 **0.85 m/s / K255 / 反向制动** 控制律的三个黄线停车档。除 `yellow_stop_duration` 外，转向、速度、视觉、反向制动参数完全一致，因此可以互换使用。

## 1. 三组参数与实测依据

| 顺序 | 参数文件 | 黄线停车 | 定位 | 已记录圈速 |
|---|---|---:|---|---|
| A（主选） | `RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD045.yaml` | 0.45 s | 当前选定 | 1 圈：20.779 s（22:29:16）；赛前建议再补 2 圈 |
| B（备份 1） | `RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD05.yaml` | 0.50 s | 黄线更稳 | 5 圈：20.523 / 20.637 / 20.675 / 20.725 / 20.737 s，中位数 20.675 s |
| C（备份 2） | `RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD04.yaml` | 0.40 s | 最快 | 1 圈：20.354 s |

数据来源：车端 `~/rdkx5_test_data/lap_times.csv`。以上圈速均为 4/4 黄线、`start_toggle_log` 起跑、正常完赛；圈速包含黄线停车时间。

同系列参考：`HOLD07`（0.70 s）有 1 圈 21.487 s，但停车时间偏长，不进入本次三组备份。

排除记录：

- 21:11 的 INVALID 是当时重复启动 `line_follower_node` 造成的，不是参数问题，不计入。
- 22:09、22:23 的 S070 系列在环岛图未完成 4/4 黄线（INVALID 或中途停止），不作为环岛备份。

## 2. 三组共用的关键参数

| 参数 | 值 |
|---|---|
| `target_speed` / `max_speed` | 0.85 / 0.85 m/s |
| `max_yaw_rate` / `kp` | 1.70 / 2.55 |
| `speed_gain_damping` | 0.7250 |
| `sharp_curve_threshold` / `sharp_curve_speed` | 0.50 / 0.20 m/s |
| `reverse_curve_speed` | 0.16 m/s |
| `line_lost_stop_time` / `line_lost_forward_speed` | 0.8 s / 0.18 m/s |
| 黄线面积阈值 / 永久停车计数 / 清除帧数 | 0.0020 / 4 / 10 |
| `enable_reverse_brake` | true |
| `reverse_speed` / `reverse_max_duration` | -0.40 m/s / 0.45 s |
| `reverse_cutoff_speed` / `reverse_abort_speed` / `reverse_odom_timeout` | 0.10 / 0.01 / 0.15 m/s·s |
| `cmd_vel_topic` | `/cmd_vel` |
| ROI `roi_top_ratio` / `roi_bottom_ratio` | 0.62 / 0.18 |

三份 YAML 各 92 个参数键。当前车端 `line_follower_node` 另声明了 3 个 curve preview 参数（默认关闭），因此这 92 键配置仍可直接加载；缺失的 3 个参数使用节点默认值。

## 3. 比赛时怎么换

1. **默认用 A（0.45 s）**。已跑 1 圈 20.779 s，赛前建议再补 2 圈；预期与 B 接近。
2. 如果现场黄线计数不稳（未到 4/4 就恢复、或黄线未清除就起步），换 **B（0.50 s）**。B 有 5 圈实测，停车余量更大。
3. 如果黄线计数稳定、但觉得停车偏长，换 **C（0.40 s）**。C 是当前最快单圈。
4. 如果问题是**丢线、压线、转向摆振**，换这三组没有用：它们的转向和速度完全相同。应降速或调 `kp`，例如改用环岛验证过的 `C11_058_curve025_y0020_hold10.yaml` 或 `HS65_065_y002_curve022.yaml` 作为保守组，再逐步恢复速度。

### 启动方式

菜单：

```bash
cd ~/aic_car_final
bash src/scripts/start_line_following.sh
# 选择 1（环岛图），再按文件名选择对应 YAML；菜单编号会变，不要只记编号。
```

直接指定文件：

```bash
bash src/scripts/start_line_following.sh --map ring \
  --params RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD045.yaml
```

把最后的文件名换成 B 或 C 即可。

## 4. 赛前验证建议

本次记录中 A 有 1 圈、B 有 5 圈、C 有 1 圈。比赛前建议：

- A：再补 2 圈（共 3 圈）；
- B、C：各 1～2 圈；
- 每圈要求 4/4 黄线、无压线、正常完赛，并用 `lap_timer.py` 记录圈速。

如果 A 的 3 圈都稳定，就按 A → B → C 的顺序使用；如果 A 出现黄线计数问题，直接把 B 作为主选。

## 5. 恢复方法

在 Mac 或车端仓库根目录执行：

```bash
bash docs/实车测试/backup_ring_20261031/restore_to_presets.sh
```

脚本会：

1. 校验本目录 `SHA256SUMS`；
2. 把 `parameter_presets/` 中同名文件先备份为 `.bak.时间戳`；
3. 把三份 YAML 复制回 `parameter_presets/`；
4. 再次校验复制后的哈希。

恢复后需要重启 `line_follower` 才生效。

## 6. 运行环境

见同目录 `ENVIRONMENT.txt`（备份时的二进制为 2026-10-31 21:10:39）。车端工作区已在 2026-10-31 23:45:59 重建，包含反向制动和 curve preview；三组参数依赖反向制动，curve preview 默认关闭。若重新部署源码并构建，先确认 `enable_reverse_brake`、`reverse_speed`、`reverse_max_duration` 等参数仍被节点声明。
