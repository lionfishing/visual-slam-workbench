# ROS 2 重构验证记录

日期：2026-10-21。环境：Ubuntu 22.04、ROS 2 Humble；使用现有固定版本核心和系统 OpenCV。

本轮直接构建并验证两个重构节点，未重新构建第三方核心。记录为单机、单序列、单次功能验收，不是精度或实时性能基准。

## 构建与运行

- 两包构建返回 0，两个包的构建 stderr 均为空。上游头文件按 SYSTEM 引入，不将其弃用警告与自写源码混在一起。
- 基线 Git 提交：`dac4d69f401f90325a1b620998f9d156bd83d0c7`。测试时工作区包含本轮尚未提交的修改，源码以文末指纹为准。
- 测试 ROS 域：201。播放器固定周期：200ms，保留数据集采集时间。
- 执行：`python3 scripts/test_ros2_rgbd.py --frames 792 --period-ms 200 --domain 201 --output-dir results/ros2-refactor-20261021`。
- 结果：**32/32 项通过**。

## 完整序列结果

| 检查项 | 结果 |
| --- | --- |
| 输入及处理 | RGB 792、深度 792、处理 792 对 |
| 配对 | 与关联文件全部按顺序精确一致，无重复 |
| 显式尾帧 | 1 对；其余 791 对由同步器输出 |
| 跟踪状态 | 本次 792 帧均报告 OK |
| 位姿订阅 | 实收 792 条，仅对应 OK 帧 |
| 消息头 | stamp 逐条精确等于对应 RGB；frame_id 为 map |
| 数值 | 位置与四元数均有限；四元数范数最大偏差 7.21e-08 |
| 退出 | 播放器与接收端均为 0，核心 Shutdown 调用出现 |

## 用例清单

另以默认 33ms 周期完整重放一次 792 对，独立 ROS 域 202：全部配对按序精确一致，792 帧报告 OK，实际收到 792 条位姿，尾帧 1 对，两端正常退出。四元数范数最大偏差约 7.02e-8。报告位于 `results/ros2-default-period-20261021/report.json`。本次未观察到缺帧，但单次成功不证明负载变化时的实时性、输入背压策略或长期稳定性。

| 用例 | 结果 |
| --- | --- |
| `timeout_zero` | 通过 |
| `timeout_negative` | 通过 |
| `timeout_nan` | 通过 |
| `timeout_inf` | 通过 |
| `missing_vocabulary` | 通过 |
| `settings_directory` | 通过 |
| `negative_max_frames` | 通过 |
| `zero_period` | 通过 |
| `bad_timestamp_0` | 通过 |
| `bad_timestamp_1` | 通过 |
| `bad_timestamp_2` | 通过 |
| `bad_timestamp_3` | 通过 |
| `empty_association` | 通过 |
| `malformed_association` | 通过 |
| `missing_rgb_image` | 通过 |
| `missing_depth_image` | 通过 |
| `leading_comments` | 通过 |
| `missing_both` | 通过 |
| `missing_rgb` | 通过 |
| `missing_depth` | 通过 |
| `excess_rgb` | 通过 |
| `excess_depth` | 通过 |
| `duplicate_finish` | 通过 |
| `sync_empty` | 通过 |
| `sync_mismatch` | 通过 |
| `sync_malformed` | 通过 |
| `tail_empty` | 通过 |
| `tail_mismatch` | 通过 |
| `tail_malformed` | 通过 |
| `not_initialized` | 通过 |
| `interrupt` | 通过 |
| `real_sequence` | 通过 |

参数用例在加载词典前拒绝非法输入。合成空图像、尺寸错误与转换错误分别覆盖同步输出和显式尾帧两条路径，失败时处理数与尾帧数均不增加。

重复结束通知用例持续发送不同计数，仍按首次期望数量和首次计时起点超时。无纹理输入保持 NOT_INITIALIZED，实际收到位姿 0 条。Ctrl+C 用例确认进入核心结束调用。

## 证据和复现

离线评估另外通过 6 项 `unittest`：已知刚体变换恢复、保留尺度误差、拒绝镜像作为旋转、拒绝退化点集、时间关联边界/复用规则，以及非法轨迹记录。现有真实轨迹重构前后的数值报告完全一致：790 对，位置 ATE RMSE 为 0.010364941318282897m。两张输出图已生成并查看；新结果位于 `results/trajectory-refactor-20261021/`，此结果不代表本次 ROS 位姿的精度。

原始报告、完整 stdout/stderr、在线位姿与临时关联文件位于 `results/ros2-refactor-20261021/`。results 目录不纳入 Git；本页保存精简结论及源码指纹。

测试前需先按 README 构建；复制旧测试脚本或只运行安装目录的旧二进制不能验证新的源码。运行脚本的参数透传与构建失败停止行为也使用命令替身检查，不重复构建。

| 文件 | SHA256 |
| --- | --- |
| `configs/tum_fr1_rgbd.yaml` | `b500ffa42112ffe82e50782a327448dc9abb770a262102d855ecb0e44ebe3719` |
| `third_party/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt` | `c2de8a268d1a4273b6764f774cc8c829b352eb1179b195078bf5180f23dce23a` |
| `ros2_ws/src/rgbd_slam/src/rgbd_slam_node.cpp` | `e5d574faffeb941f4985849e0b8f4618719c346180e9c3643cfe96117cc3031b` |
| `ros2_ws/src/rgbd_dataset_player/src/player_node.cpp` | `08ceedc4b080fa38d6e79a4963bfd1282a5e86618c435ecef0bc18c9a623ee8a` |

## 尚未由本次测试证明的事项

- 33ms 周期的无丢帧能力、端到端延迟、实时性能和长时间运行稳定性。
- 位姿 ATE/RPE、多序列多次重复性、回环后最终轨迹精度。
- 成功跟踪后进入 RECENTLY_LOST/LOST 的全部恢复路径。
- 上游后台优化线程已经全部 join、最终轨迹可以安全导出。
- 新机器上从零安装系统和全套第三方依赖的干净复现。

## 离线评估与测试入口指纹

| 文件 | SHA256 |
| --- | --- |
| `scripts/trajectory_eval.py` | `c8d6978fd4f22a682ef7473f831959ee76880731fdb4cb8385ed8840d8c8c42a` |
| `scripts/test_trajectory_eval.py` | `5da50da8933423770b75cac93393fd7d9d5e07e87db66e7a4ae05df54ee6b6c2` |
| `scripts/test_ros2_rgbd.py` | `828746590f09bdb0004f7b8e30ec4cbdb81db4c6821ad52199b6200f7b06e9a3` |
