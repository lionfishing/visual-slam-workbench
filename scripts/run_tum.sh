#!/usr/bin/env bash
set -e
slam_project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
echo "项目目录: $slam_project_root"
# 确保结果总目录存在 - mkdir -p：目录不存在就创建，已经存在也正常继续
mkdir -p "$slam_project_root/results"
# TODO：使用 mktemp -d 创建唯一运行目录，并保存到 slam_run_dir
slam_run_dir=$(mktemp -d "$slam_project_root/results/tum-fr1-xyz-XXXXXXXX")
echo "本次结果目录：$slam_run_dir"

# TODO：切换到本次结果目录
cd "$slam_run_dir"
pwd
"$slam_project_root/third_party/ORB_SLAM3/Examples/RGB-D/rgbd_tum" \
  "$slam_project_root/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "$slam_project_root/third_party/ORB_SLAM3/Examples/RGB-D/TUM1.yaml" \
  "$slam_project_root/data/rgbd_dataset_freiburg1_xyz" \
  "$slam_project_root/third_party/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt" \
  > run.log 2>&1

echo "SLAM 运行结束，开始评估"
"$slam_project_root/.venv/bin/python" \
  "$slam_project_root/scripts/trajectory_eval.py" \
  --estimated "$slam_run_dir/CameraTrajectory.txt" \
  --groundtruth "$slam_project_root/data/rgbd_dataset_freiburg1_xyz/groundtruth.txt" \
  --max-difference 0.02 \
  --output-dir "$slam_run_dir"

echo "运行与评估完成：$slam_run_dir"