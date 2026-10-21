# 构建与数据准备

以下说明针对 Ubuntu 22.04、ROS 2 Humble 和本仓库的固定上游版本。命令均从项目根目录执行，依赖配置使用项目内安装前缀。本轮在已有依赖环境中重新构建了两个 ROS 包；没有重新执行操作系统安装和全套第三方干净构建。

## 系统依赖

已有 ROS Humble 环境需提供 `rclcpp`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`message_filters`、`cv_bridge`、`ament_cmake` 和 colcon。

核心还使用系统 OpenCV 4、Eigen 3、Boost serialization、OpenSSL、OpenGL、GLEW、PNG/JPEG 及基础 C++ 构建工具。测试脚本使用系统 Python 和 `rclpy`，不需要单独安装 pytest。

根据机器现状通过 rosdep 或系统包管理器补齐缺项。不要为此建立另一套 PyTorch/CUDA 环境。开始下载前检查磁盘，保留足够的编译和数据解压空间。

## 固定源码

仅在相应目录不存在时克隆，已有源码先核对 `git rev-parse HEAD`，避免覆盖本地修改：

```bash
mkdir -p third_party build install
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git third_party/ORB_SLAM3
git -C third_party/ORB_SLAM3 checkout 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
git clone https://github.com/stevenlovegrove/Pangolin.git third_party/Pangolin
git -C third_party/Pangolin checkout dd801d244db3a8e27b7fe8020cd751404aa818fd
```

来源与版本同时保存在 `dependencies.json`。使用 ORB-SLAM3 自带的 DBoW2、g2o 和 Sophus，不替换为另一套系统版本。

## 项目内依赖构建

以下选项由本机既有 CMake 配置整理，建议从两个并行任务开始：

```bash
cmake -S third_party/Pangolin -B build/pangolin \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/pangolin" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PYPANGOLIN_MODULE=OFF \
  -DBUILD_PANGOLIN_LIBOPENEXR=OFF -DBUILD_PANGOLIN_LIBTIFF=OFF
cmake --build build/pangolin -j2
cmake --install build/pangolin

cmake -S third_party/ORB_SLAM3/Thirdparty/DBoW2 -B build/dbow2 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14
cmake --build build/dbow2 -j2

cmake -S third_party/ORB_SLAM3 -B build/orbslam3 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG" \
  -DPangolin_DIR="$PWD/install/pangolin/lib/cmake/Pangolin"
cmake --build build/orbslam3 --target ORB_SLAM3 rgbd_tum -j2
```

主构建会构建其自带 g2o。词典若仍是压缩包，在词典目录解压 `ORBvoc.txt.tar.gz`，保留 `Vocabulary/ORBvoc.txt` 供节点使用。

## 数据

当前使用 Freiburg 1 XYZ：

```text
https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
```

将归档解压到 `data/`，使图像位于 `data/rgbd_dataset_freiburg1_xyz/rgb/` 和 `depth/`。关联文件使用上游仓库自带的 `Examples/RGB-D/associations/fr1_xyz.txt`，本机共有 792 对。

历史下载归档 SHA256 为 `a0236d97b8c30cd93b653656d2b6c293ff7c982a4130ef2a1a8beecdb124ef98`，这是本地归档指纹，不是已验证的官方校验值。数据文件、第三方源码和词典不纳入 Git。

## ROS 包

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select rgbd_dataset_player rgbd_slam --parallel-workers 2
source install/local_setup.bash
```

接收包从自身目录计算项目根路径，显式使用项目内 Pangolin。CMake 的 `INSTALL_RPATH` 为安装节点保留核心和 Pangolin 的库目录；`ros2 run` 启动的是安装程序，只有构建目录中存在库搜索路径并不足够。

项目移动目录后应重新配置/构建依赖和 ROS 包。出现找不到库时先加载 ROS 环境，用 `ldd`/`readelf -d` 核对实际安装程序；不需要把项目库复制到系统目录。

运行和测试命令见 [README](../README.md)。

## Python 轨迹评估

此虚拟环境只用于离线评估，不用于 ROS 构建或 `rclpy` 测试：

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

依赖固定在 `requirements.txt`，当前包含 NumPy 和 Matplotlib。
