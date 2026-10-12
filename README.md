# ROS 2 RGB-D 视觉定位、建图与评估系统

用于 SLAM / 机器人软件实习作品的长期项目，核心基于 ORB-SLAM3，个人模块将覆盖 ROS 2 数据接口、状态诊断、关键帧点云和实验评估。

已有基线：源码固定、项目内依赖构建、首个 TUM RGB-D 序列运行与轨迹导出，以及单次轨迹的初步位置 ATE。尚未完成重复评估、RPE、ROS 2 集成和点云模块；当前基线是普通 C++ 程序。

当前开发入口：[scripts/trajectory_eval.py](scripts/trajectory_eval.py)，由用户逐步实现真实轨迹读取与绘图。在项目根目录运行：

```bash
    .venv/bin/python scripts/trajectory_eval.py \
    --estimated results/tum-fr1-xyz-gXqAVwwv/CameraTrajectory.txt \ 
    --groundtruth data/rgbd_dataset_freiburg1_xyz/groundtruth.txt \
    --max-difference 0.02 \
    --output-dir results/evaluation
```

默认输出保存到估计轨迹所在目录，包括 aligned_xz.png、position_error.png 和 evaluation.json；可通过 --output-dir 指定其他目录。`docs/` 集中存放说明文档，`learning/` 保存学习练习。`third_party/`、`build/`、`install/` 是已准备的 C++ 基线，`.venv/` 是当前 Python 环境；目录按实际需求逐步增加。

- [项目方案](docs/PROJECT_PLAN.md)
- [当前进度](PROGRESS.md)
- [已有仓库阅读记录](docs/REPOSITORY_REVIEW.md)

实际构建依赖版本见 dependencies.json；上游算法能力与个人实现分开描述，不预填精度或性能成绩。第三方、构建、数据与运行目录默认忽略，未来发布仓库时保留版本清单和复现步骤。

目录随实际需求逐步搭建。`scripts/` 存放处理真实项目数据的工具，过时教程和旧构建日志已清理，历史结论与验证边界保留在 PROGRESS.md；当前已实现位置 ATE，尚未实现 RPE

## 评估工具第一版完成 

- aligned_xz.png：刚体对齐后，估计轨迹与真值轨迹在 X–Z 平面上的投影对比。
- position_error.png：三维位置误差随时间变化，不是 X–Z 平面误差。误差同时包含 X、Y、Z 三个分量。
- evaluation.json：评估条件与统计结果，包括输入路径、时间差阈值、关联数量、对齐方式和误差指标。

输入为估计轨迹与真值轨迹的 TUM 格式记录文件。程序按时间关联对应记录，并进行不缩放的刚体对齐，输出 X–Z 平面轨迹对比图、三维位置误差时间曲线和 JSON 评估报告。本次单次轨迹评估关联 790 对记录，位置 ATE RMSE 约为 1.04 厘米。同一输出目录重复运行会更新同名文件。


