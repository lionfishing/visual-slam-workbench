# ROS 2 RGB-D 视觉定位、建图与评估系统

基于固定版本 ORB-SLAM3 和 ROS 2 Humble 的学习与实验项目。上游提供 SLAM 核心；本仓库实现数据播放、ROS 2 接收与同步、位姿发布、结束协议和轨迹评估工具。

当前已接通真实 TUM RGB-D 图像到 `/slam/pose` 的数据流。支持无窗口运行，发布相机相对于当前 SLAM 地图的在线位姿。点云建图、RViz 配置、TF、ROS 轨迹导出及 RPE 仍待实现。

## 数据流与代码

```text
TUM 图像和关联文件
        ↓
rgbd_dataset_player（固定周期，保留采集时间戳）
        ↓ RGB / 深度 / CameraInfo / 结束通知
rgbd_slam（近似同步 → 图像检查 → TrackRGBD）
        ↓ 仅跟踪状态为 OK 时发布
/slam/pose（PoseStamped）
```

| 位置 | 职责 |
| --- | --- |
| `ros2_ws/src/rgbd_dataset_player` | 读取关联文件、解析时间戳、构造和发布图像及标定消息 |
| `ros2_ws/src/rgbd_slam` | RGB-D 同步、核心生命周期、状态判断、位姿与结束处理 |
| `configs/tum_fr1_rgbd.yaml` | Freiburg 1 标定、BGR 输入、深度比例 5000 |
| `scripts/run_rgbd_*.sh` | 构建并运行对应节点，支持附加 ROS 参数 |
| `scripts/test_ros2_rgbd.py` | 独立 ROS 域中的可重复端到端验证 |
| `scripts/trajectory_eval.py` / `scripts/test_trajectory_eval.py` | 位置 ATE 评估及数学/输入边界回归 |
| `dependencies.json` | 上游源码地址、固定提交与安装位置 |

节点采用单线程 executor。类成员持有订阅者、同步器、定时器、核心和处理状态；回调通过 `[this]` 调用具名函数。结构与测试说明见 [重构说明](docs/REFACTORING.md)。

## 环境和准备

已在 Ubuntu 22.04、ROS 2 Humble、GCC 11、系统 OpenCV 4.5.4 上构建。自写节点使用 C++17。ORB-SLAM3 和 Pangolin 使用项目内的固定源码/安装路径，不安装到系统目录。

运行前应存在：

```text
third_party/ORB_SLAM3/lib/libORB_SLAM3.so
third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
third_party/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt
install/pangolin/lib/libpangolin.so
data/rgbd_dataset_freiburg1_xyz/
```

这些大文件不随 Git 上传。源码、依赖和数据的准备步骤见 [构建说明](docs/BUILDING.md)。相机信息目前固定为 TUM Freiburg 1 的原始 640×480 分辨率；更换相机或序列标定时需同步调整播放器和 SLAM 配置。

## 运行 ROS 2 RGB-D SLAM

以下脚本从任意工作目录调用都可以，示例在项目根目录执行。确保没有上一轮接收节点仍在监听同名话题。

终端 A，构建并启动接收节点：

```bash
bash scripts/run_rgbd_slam.sh
```

首次加载约 145MB 的词典需要数秒，看到 `Vocabulary loaded!` 后继续等待输入。默认关闭 Pangolin 窗口。

终端 B，观察一条位姿：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /slam/pose geometry_msgs/msg/PoseStamped --once
```

终端 C，先用慢速短序列：

```bash
bash scripts/run_rgbd_player.sh -p max_frames:=30 -p publish_period_ms:=200
```

播放器等待 RGB、深度和结束通知三个订阅端匹配后才开始。完整的正常运行会在两端自动退出；正常退出码为 0，参数、文件、图像或收尾错误返回 1。`echo --once` 收到一条就退出，不影响后续 SLAM 处理。

完整序列的保守功能验证入口：

```bash
bash scripts/run_rgbd_player.sh -p max_frames:=0 -p publish_period_ms:=200
```

`max_frames=0` 表示全部记录。播放器脚本原有默认值仍为全部记录、33ms 周期；这不是“完整不丢帧模式”的承诺。SLAM 在回调中同步执行，两路订阅和同步器缓存深度均为 10，处理变慢可能丢失或错过配对。固定周期改变发送节奏，不改变消息中的采集时间，也不是按原始帧间隔精确回放。

### 话题与数据约定

| 话题 | 消息类型 | 含义 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | BGR8，RGB 原始采集时间 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 16UC1，深度原始采集时间，已配准到彩色相机 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色相机标定，消息头与 RGB 一致 |
| `/dataset/playback_finished` | `std_msgs/msg/UInt64` | 本轮播放器实际发布的图像对数 |
| `/slam/pose` | `geometry_msgs/msg/PoseStamped` | 当前地图中相机的在线位置与朝向 |

- 接收端按 20ms 上限近似配对；这不补偿两路采集时间差中的运动。
- `TrackRGBD` 使用 RGB 时间。输出位姿沿用该时间戳，`frame_id=map`。
- 上游返回世界到相机的 `T_cw`，发布前求逆得到 `T_wc`。四元数显式填写 `x/y/z/w`。
- `map` 是当前 SLAM 地图的名字，不自动成为重力对齐、Z 向上的机器人世界坐标系。输出表示相机 optical 坐标的位姿，尚未发布 TF 或 `base_link` 外参。
- `Camera.RGB=0` 匹配 BGR 输入。整数深度保留原值，由核心按 `RGBD.DepthMapFactor=5000` 转换成米，外部不再次缩放。
- 只有 `Tracking::OK` 才发布位姿。处理调用计数不等于有效跟踪计数；不伪造协方差。

### 收尾协议与参数

接收端处理单次播放。首次结束通知固定期望数量和稳态时钟起点，重复通知不覆盖它们。计数超额直接失败；不足时等待 `finish_timeout_sec`，到期失败。输入收齐后，如果同步器只保留最后一对，则检查它未被处理且时间差不超过 20ms，再显式处理一次。

`Summary` 中 `pairs` 是处理完成总数，包含 `tail_pairs`；不能把两者相加。可靠传输确认仅表示 DDS 层确认，不代表接收端已经完成核心处理。

| 节点参数 | 默认值 | 约束 |
| --- | --- | --- |
| 播放器 `dataset_root` / `association_path` | 空，脚本提供项目路径 | 数据目录和关联文件有效 |
| 播放器 `wait_for_subscribers` | `true` | 等待三路匹配订阅者 |
| 播放器 `max_frames` | 节点 3；脚本 0 | 非负；0 表示全部 |
| 播放器 `publish_period_ms` | 节点 1000；脚本 33 | 正整数 |
| 接收端 `vocabulary_path` / `settings_path` | 空，脚本提供项目路径 | 必须是普通文件 |
| 接收端 `finish_timeout_sec` | 5.0 | 正且有限；从首次完成通知开始计时 |

没有收到结束通知时，接收端会继续等待，例如播放器中途异常退出的情况；可使用 Ctrl+C。该超时不是整个播放过程的 watchdog。

## 自动验证

先构建，然后使用系统 Python（不要激活视觉虚拟环境）：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select rgbd_dataset_player rgbd_slam --parallel-workers 2
source install/local_setup.bash
cd ..
python3 scripts/test_ros2_rgbd.py
```

默认在 ROS 域 200 测试 30 对慢速真实图像，并覆盖非法参数、关联文件/时间戳错误、缺失图像、输入不足/超额、重复通知、图像转换/形状错误、未初始化不发布和 Ctrl+C。请选择未被其他任务使用的测试域。

完整序列：

```bash
python3 scripts/test_ros2_rgbd.py --frames 792 --period-ms 200 --domain 201
```

结果写入新的 `results/ros2-validation-*/`：`report.json` 含各项结果、源码指纹、Git 状态、配置及上游版本；`poses.json` 保存订阅收到的在线位姿，其余日志保留失败原因和每帧记录。测试不会停止用户自行启动的节点。

已构建后只回归真实数据，可加 `--only-real`，例如单独检查默认播放节奏：

```bash
python3 scripts/test_ros2_rgbd.py --only-real --frames 792 --period-ms 33 --domain 202
```

实际验收记录及结论边界见 [验证记录](docs/VALIDATION.md)。

## 离线基线与位置 ATE

```bash
bash scripts/run_tum.sh
```

该入口创建独立的 `results/tum-fr1-xyz-*/` 目录，运行上游 RGB-D 示例，再调用评估工具。需要先按构建说明准备核心、数据和 Python 评估环境。

也可以单独评估已有的 TUM 格式轨迹：

```bash
.venv/bin/python scripts/trajectory_eval.py \
  --estimated results/<本次运行>/CameraTrajectory.txt \
  --groundtruth data/rgbd_dataset_freiburg1_xyz/groundtruth.txt \
  --max-difference 0.02 \
  --output-dir results/<本次评估>
```

输出 `aligned_xz.png`、`position_error.png` 和 `evaluation.json`。位置误差使用三维位置，SE(3) 对齐不缩放；同一评估目录重复运行会更新同名文件。该离线导出轨迹和 ROS 在线位姿是不同结果，不直接混用其精度结论。

时间关联保持原工具的规则：对每条估计记录寻找最近真值，允许复用真值记录，无插值、无时间偏移；这不是全局一对一关联。对齐拒绝重合或共线的退化位置，不使用缩放消除尺度误差。评估函数的合成回归可单独运行：

```bash
.venv/bin/python scripts/test_trajectory_eval.py
```

## 已知边界与后续工作

- 尚未实现有界业务队列和明确的全量/实时丢帧策略，默认 33ms 播放需单独评估。
- 固定上游 `Shutdown()` 的线程等待段被注释。本节点请求核心结束并退出，但尚未验证所有后台优化都完成；当前 ROS 节点不导出最终优化轨迹。
- 相机标定、地图重置/回环后的全局一致性、失效状态通知、仿真时钟与 rosbag2 回放还需继续完善。
- 位置 ATE 工具已实现；ROS 位姿精度、RPE、多序列多次重复实验、关键帧点云、RViz、TF 和实时性能仍需独立验收。
- 许可证字段暂待项目作者确定。使用和分发 ORB-SLAM3 等第三方内容时应保留其许可证及来源；本轮未替作者选择新的许可证。

完整路线见 [项目方案](docs/PROJECT_PLAN.md)。
