# AIC-car-final

RDK X5 ROS 2 小车的底盘、视觉巡线、参数预设和现场工具。

## 目录

- `src/car_ws`：Wheeltec 底盘 ROS 2 工作区
- `src/line_follow_ws`：C++ 巡线和可选 YOLO 数字转向工作区
- `src/camera`、`src/yolov8`：相机与 YOLO 节点
- `src/scripts`：一键启动和计时工具
- `docs`：编译、调试与实车记录

## RDK X5 编译

```bash
source /opt/tros/humble/setup.bash
cd src/car_ws && colcon build --symlink-install
source install/setup.bash
cd ../line_follow_ws && colcon build --symlink-install
```

## 巡线启动

从工作区根目录运行 `bash src/scripts/start_line_following.sh`，默认不启动 YOLO。指定地图和参数时，例如：

```bash
bash src/scripts/start_line_following.sh --map ring \
  --params parameter_presets/RB_FORK85_085_R040_P045_K255_Y170_Y020_HOLD05.yaml
```

计时器在另一个终端运行：

```bash
source /opt/tros/humble/setup.bash
python3 src/scripts/lap_timer.py --label ring-test
```

更完整的板端环境与调试步骤见 [`src/RDK_X5_小车编译_巡线调参与二次开发完整说明.md`](src/RDK_X5_小车编译_巡线调参与二次开发完整说明.md)。
