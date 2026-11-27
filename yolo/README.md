# AIC 智能车 YOLO 多版本模型仓库

本目录按版本归档了所有决赛数字识别模型（输入 `640x640` letterbox `NV12`，输出 RDK X5 `bayes-e` 6 头标准格式 `1~8`），`src/` 原路径文件保持不动。

| 目录 | 文件名 | 架构与精度 | 大小 | MD5 | 训练数据与用途说明 | 状态 |
|---|---|---|---|---|---|---|
| `v0_orig_11m` | `best_modified_yolov8_orig_11m.bin` | YOLOv8s 量级 INT8 BPU | 11M | `8d6f45ebecdfad1c8dfca1eb561b508e` | 原赛题基线模型（`rdkx5_bench` `finals_val mAP@0.5=0.1208`、`39.3 FPS`；标注风格偏内圈白字） | 原始基线备份 |
| `v1_yolov8n` | `best_yolov8n_v1.pt` | YOLOv8n FP32 PyTorch | 6.2M | `5e70e780b13eb75730b916613bd7e884` | 初版合成微调 PyTorch 权重（对应首版 RDK 导出基座） | 历史归档 |
| `v2_yolov8n` | `best_modified_yolov8_v2.bin` | YOLOv8n 纯 INT8 BPU | 3.5M | `b75634e65ea7249f74a431db56b1d79e` | 仅合成+空赛道背景训练（`finals_val mAP@0.5=0.6075`、`62.9 FPS`；真场地缺实景样本） | 历史可用基线 |
| `v2_yolov8n` | `best_modified_yolov8_v2_int16.bin` | YOLOv8n `kl`+部分`int16` | 3.6M | `7e4a45717da88f3b903319aba72b583a` | 实验性 `int16` 检出头版本（实测 `mAP@0.5` 跌至 `0.3956`、倒置 `6` 归零） | 已废弃 |
| `v3_yolov8n` | `best_modified_yolov8_v3.bin` | YOLOv8n 纯 INT8 BPU | 3.5M | `3cec559bc8b2b0970aeb5384427fd68e` | `v2` + 单圈真包 `final_030`（119 帧）+ 1800 合成（`finals_val mAP@0.5=0.6207`、`59.8 FPS`；缺右斜角 `1`） | 上一版实测模型 |
| `v5_yolov8s` | `best_modified_yolov8s_v5.bin` | **YOLOv8s 纯 INT8 BPU** | 11M | `bd98b8b5baedd286826e783ce233a5d3` | 全部 22 个真车录包人工清洗统一整圆标框（`601 train` 4× 过采样，~67% 真图 + `1200` 合成回放；100% BPU 算子，6 头余弦相似度 `0.9897–0.9999`） | **最新主推版（待上车实测）** |
| `v5_yolov8s` | `best_yolov8s_v5.pt` | YOLOv8s FP32 PyTorch | 21M | `340ef7ea92281aea94eb5be59c2419e6` | `v5` Colab T4 训练 `best.pt` 原权重 | 云端导出源权重 |

## `v5_yolov8s` 评测与量化摘要

- **FP32 6 头 ONNX 离线评测（`v5_yolov8s/eval_fp32.json`）**：
  - 整包隔离真场地日间（`fork_04` / `real_day_pos`）：`mAP@0.5 = 0.9874`
  - 整包隔离真场地夜间（`ring_02` / `real_night_pos`）：`mAP@0.5 = 0.9890`
  - 混合验证集总计（含合成难例 `3`/`6`/倒置 `6`/小目标 + 真场 `END`/锥桶/弯道强负样本）：`mAP@0.5 = 0.7577`，`P = 0.6820`，`R = 0.9724`，`Inverted-6 = 24/36 (66.7%)`
- **纯 INT8 PTQ 6 头输出余弦相似度（`v5_yolov8s/hb_mapper.log`）**：
  - `cls_8`: `0.997171` | `reg_8`: `0.990258`
  - `cls_16`: `0.997634` | `reg_16`: `0.992600`
  - `cls_32`: `0.999927` | `reg_32`: `0.989711`
