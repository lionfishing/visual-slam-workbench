# ROS 2 RGB-D 视觉定位、建图与评估系统

用于 SLAM / 机器人软件实习作品的长期项目，核心基于 ORB-SLAM3，个人模块将覆盖 ROS 2 数据接口、状态诊断、关键帧点云和实验评估。

已完成固定版本 ORB-SLAM3 离线基线及位置 ATE 评估，并实现 ROS 2 RGB-D 数据播放、图像同步、SLAM 核心调用和带时间戳位姿发布。ROS 2 链路已通过 30 对真实图像的慢速短序列测试，并验证未初始化时不发布位姿。接入 SLAM 后的完整序列、实时性能、RPE 和点云建图尚未完成验收。

当前开发入口：[scripts/trajectory_eval.py](scripts/trajectory_eval.py)，由用户逐步实现真实轨迹读取与绘图。在项目根目录运行：

```bash
    .venv/bin/python scripts/trajectory_eval.py \
    --estimated results/tum-fr1-xyz-gXqAVwwv/CameraTrajectory.txt \
    --groundtruth data/rgbd_dataset_freiburg1_xyz/groundtruth.txt \
    --max-difference 0.02 \
    --output-dir results/evaluation
```

默认输出保存到估计轨迹所在目录，包括 aligned_xz.png、position_error.png 和 evaluation.json；可通过 --output-dir 指定其他目录。`docs/` 集中存放说明文档，`third_party/`、`build/`、`install/` 是已准备的 C++ 基线，`.venv/` 是当前 Python 环境；目录按实际需求逐步增加。

- [项目方案](docs/PROJECT_PLAN.md)

实际构建依赖版本见 dependencies.json；上游算法能力与个人实现分开描述，不预填精度或性能成绩。第三方、构建、数据与运行目录默认忽略，未来发布仓库时保留版本清单和复现步骤。

目录随实际需求逐步搭建。`scripts/` 存放处理真实项目数据的工具，过时教程和旧构建日志已清理，历史结论与验证边界保留在 PROGRESS.md；当前已实现位置 ATE，尚未实现 RPE

## 评估工具第一版完成

- aligned_xz.png：刚体对齐后，估计轨迹与真值轨迹在 X–Z 平面上的投影对比。
- position_error.png：三维位置误差随时间变化，不是 X–Z 平面误差。误差同时包含 X、Y、Z 三个分量。
- evaluation.json：评估条件与统计结果，包括输入路径、时间差阈值、关联数量、对齐方式和误差指标。

输入为估计轨迹与真值轨迹的 TUM 格式记录文件。程序按时间关联对应记录，并进行不缩放的刚体对齐，输出 X–Z 平面轨迹对比图、三维位置误差时间曲线和 JSON 评估报告。本次单次轨迹评估关联 790 对记录，位置 ATE RMSE 约为 1.04 厘米。同一输出目录重复运行会更新同名文件。

## 运行 RGB-D 基线

- 运行前需要准备好 ORB-SLAM3、数据集和项目虚拟环境。
- 在项目根目录运行：

```bash
bash scripts/run_tum.sh
```
脚本会创建独立结果目录，运行 ORB-SLAM3，并自动调用轨迹评估工具。
- 每次结果保存在新的 results/tum-fr1-xyz-XXXXXXXX/ 目录中。

## ROS 2 RGB-D 数据播放
- 运行入口：bash scripts/run_rgbd_player.sh。
- 需要准备 TUM 数据集及关联文件。
- 两个图像话题及对应的 echo 命令，要在不同终端运行。
- 默认等待两个订阅者连接；脚本播放全部 792 对，固定周期 33ms，消息保留原始采集时间。
- 当前入口只播放数据，不会启动 SLAM。
- 数据集:data/rgbd_dataset_freiburg1_xyz/
- 关联文件:third_party/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt

终端1：订阅彩色图像
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /camera/color/image_raw sensor_msgs/msg/Image --field header
```

终端2：订阅深度图像
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /camera/depth/image_raw sensor_msgs/msg/Image --field header
```

终端3：在项目根目录启动播放器
```bash
bash scripts/run_rgbd_player.sh
```
