# ROS 2 RGB-D 视觉定位、建图与评估系统

用于 SLAM / 机器人软件实习作品的长期项目，核心基于 ORB-SLAM3，个人模块将覆盖 ROS 2 数据接口、状态诊断、关键帧点云和实验评估。

已有基线：源码固定、项目内依赖构建、首个 TUM RGB-D 序列运行与轨迹导出，以及单次轨迹的初步位置 ATE。尚未完成重复评估、RPE、ROS 2 集成和点云模块；当前基线是普通 C++ 程序。

当前开发入口：[scripts/trajectory_eval.py](scripts/trajectory_eval.py)，实现轨迹读取、时间关联、不缩放刚体位置对齐、位置 ATE、绘图和 JSON 报告。

使用 Python 3.10，在项目根目录创建独立环境并安装直接依赖（已有环境不必重建）：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

在项目根目录运行：

```bash
.venv/bin/python scripts/trajectory_eval.py \
  --estimated results/tum-fr1-xyz-gXqAVwwv/CameraTrajectory.txt \
  --groundtruth data/rgbd_dataset_freiburg1_xyz/groundtruth.txt \
  --max-difference 0.02 \
  --output-dir results/evaluation
```

命令使用本机已有的首次运行结果。仓库不包含数据集与该轨迹；其他使用者需准备 TUM 格式估计轨迹和真值，再通过参数传入。默认输出保存到估计轨迹所在目录，包括 `aligned_xz.png`、`position_error.png` 和 `evaluation.json`；可通过 `--output-dir` 指定其他目录。

当前目录用途：

| 路径 | 用途 | 纳入 Git |
| --- | --- | --- |
| `scripts/` | 自写评估代码 | 是 |
| `docs/PROJECT_PLAN.md` | 项目计划 | 是 |
| `requirements.txt` | Python 直接依赖版本 | 是 |
| `dependencies.json` | 第三方 C++ 源码版本 | 是 |
| `.venv/` | 本地 Python 环境 | 否 |
| `third_party/`、`build/`、`install/` | 第三方源码和构建产物 | 否 |
| `data/`、`results/` | 本地输入与实验输出 | 否 |

- [项目方案](docs/PROJECT_PLAN.md)

实际构建依赖版本见 dependencies.json；上游算法能力与个人实现分开描述，不预填精度或性能成绩。第三方、构建、数据与运行目录默认忽略，未来发布仓库时保留版本清单和复现步骤。

目录随实际需求逐步搭建。详细学习进度和旧仓库审阅记录留在本地，不纳入 Git。当前已实现位置 ATE，尚未实现 RPE。

## 评估输出

- aligned_xz.png：刚体对齐后，估计轨迹与真值轨迹在 X–Z 平面上的投影对比。
- position_error.png：三维位置误差随时间变化，不是 X–Z 平面误差。误差同时包含 X、Y、Z 三个分量。
- evaluation.json：评估条件与统计结果，包括输入路径、时间差阈值、关联数量、对齐方式和误差指标。

输入为估计轨迹与真值轨迹的 TUM 格式记录文件。程序按时间关联对应记录，并进行不缩放的刚体对齐，输出 X–Z 平面轨迹对比图、三维位置误差时间曲线和 JSON 评估报告。本次单次轨迹评估关联 790 对记录，位置 ATE RMSE 约为 1.04 厘米。同一输出目录重复运行会更新同名文件。

当前使用逐条独立最近邻关联；本次数据中真值记录没有重复使用，但算法尚未保证任意输入的一对一关联。该结果仅来自已有单次导出轨迹，导出线程同步与重复性仍待检查。


