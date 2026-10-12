from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import json
import argparse

def load_trajectory(path):
    """读取并检查一份 TUM 格式轨迹。"""
    trajectory = np.loadtxt(path, ndmin=2)  # 使用 np.loadtxt，保留 ndmin=2

    # 如果没有任何元素，报错
    if trajectory.size == 0:
        raise ValueError(f"{path}：轨迹文件没有数值记录。")

    # 原来的 8 列检查保留
    if trajectory.shape[1] != 8:
        raise ValueError(
        f"{path}：轨迹应为 8 列，实际为 {trajectory.shape[1]} 列。"
    )

    # 如果包含 NaN 或 Inf，报错
    if not np.isfinite(trajectory).all():
        raise ValueError(f"{path}：轨迹包含 NaN 或 Inf。")

    timestamps = trajectory[:, 0]
    time_differences = np.diff(timestamps)

    if not np.all(time_differences > 0):
        raise ValueError(f"{path}：时间戳必须严格递增，不能重复或倒序。")
    return trajectory

def main():
    # 从脚本位置找到项目根目录
    project_root = Path(__file__).resolve().parent.parent

    parser = argparse.ArgumentParser(description="轨迹关联与位置 ATE 评估")
    parser.add_argument(
        "--max-difference",
        type=float,
        default=0.02,
        help="最大关联时间差，单位：秒，默认 0.02",
    )
    parser.add_argument(
        "--estimated",
        type=Path,
        default=project_root / "results" / "tum-fr1-xyz-gXqAVwwv" / "CameraTrajectory.txt",
        help="估计轨迹文件路径",
    )
    parser.add_argument(
        "--groundtruth",
        type=Path,
        default=project_root / "data" / "rgbd_dataset_freiburg1_xyz" / "groundtruth.txt",
        help="真值轨迹文件路径",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="结果目录，默认使用估计轨迹所在目录",
    )
    args = parser.parse_args()

    if not np.isfinite(args.max_difference) or args.max_difference < 0:
        parser.error("--max-difference 必须是有限的非负秒数")

    # 预估轨迹提取
    trajectory_path_estimated = args.estimated.resolve()
    trajectory_estimated = load_trajectory(trajectory_path_estimated)

    # 时间戳与位置
    timestamps_estimated = trajectory_estimated[:,0]
    positions_estimated = trajectory_estimated[:,1:4]
    trajectory_path_truth = args.groundtruth.resolve()
    trajectory_truth = load_trajectory(trajectory_path_truth)

    # 时间戳与位置
    timestamps_truth = trajectory_truth[:,0]
    positions_truth = trajectory_truth[:,1:4]

    max_time_difference = args.max_difference
    matches = []

    for estimated_index, timestamp in enumerate(timestamps_estimated):
        # 计算全部真值时间戳与当前 timestamp 的绝对差
        differences = np.abs(timestamps_truth - timestamp)
        # 找到最小时间差的索引
        nearest_index = np.argmin(differences)

        if differences[nearest_index] <= max_time_difference:
            matches.append((estimated_index, nearest_index))

    if len(matches) < 3:
        raise ValueError(
            f"只有 {len(matches)} 对时间关联，无法进行当前三维刚体对齐。"
            "请检查轨迹时间范围和关联阈值。"
        )

    # 列表转换成整数数组，每行保存一对索引
    match_indices = np.array(matches, dtype=int)

    # 取第 0 列，得到估计记录索引
    estimated_indices = match_indices[:, 0]

    # 取第 1 列，得到真值记录索引
    truth_indices = match_indices[:, 1]

    matched_estimated = positions_estimated[estimated_indices]
    matched_truth = positions_truth[truth_indices]

    # 分别计算两组关联位置的中心
    estimated_center = matched_estimated.mean(axis=0)
    truth_center = matched_truth.mean(axis=0)

    # 每个位置减去所属组的中心
    X = matched_estimated - estimated_center
    Y = matched_truth - truth_center

    # 汇总对应点关系
    H = X.T @ Y

    # 奇异值分解
    U, singular_values, Vt = np.linalg.svd(H)

    # 排除镜像变换
    D = np.eye(3)
    if np.linalg.det(Vt.T @ U.T) < 0:
        D[2, 2] = -1

    # 从估计坐标系转到真值坐标系的旋转
    R = Vt.T @ D @ U.T

    # 让旋转后的估计中心与真值中心重合
    t = truth_center - R @ estimated_center

    # 对原始关联位置先旋转，再平移
    aligned_estimated = matched_estimated @ R.T + t

    print("平移量：", t)
    print("对齐后位置形状：", aligned_estimated.shape)
    print(
        "中心是否重合：",
        np.allclose(aligned_estimated.mean(axis=0), truth_center)
    )

    # 每对位置的三维坐标差
    residuals = aligned_estimated - matched_truth

    # 每一行三维差向量的长度，单位：米
    errors = np.linalg.norm(residuals, axis=1)

    # 位置误差的均方根
    ate_rmse = np.sqrt(np.mean(errors ** 2))

    print("位置误差数量：", errors.shape)
    print("ATE RMSE(米）：", ate_rmse)
    print("ATE RMSE(厘米）：", ate_rmse * 100)

    if args.output_dir is None:
        output_dir = trajectory_path_estimated.parent
    else:
        output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "aligned_xz.png"
    error_plot_path = output_dir / "position_error.png"
    report_path = output_dir / "evaluation.json"

    # 绘图
    fig, ax = plt.subplots()

    # 横轴取位置的 X 分量，纵轴取 Z 分量
    ax.plot(
        matched_truth[:, 0],
        matched_truth[:, 2],
        label="Ground truth"
    )
    ax.plot(
        aligned_estimated[:, 0], 
        aligned_estimated[:, 2],
        label="Aligned estimate",
        linestyle="--"
    )
    ax.legend()

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title("Aligned trajectories: X-Z projection")
    ax.set_aspect("equal")
    ax.grid(True)

    fig.savefig(output_path, dpi=150)
    plt.close(fig)

    print("图片已保存：", output_path)

    # 只取参与评估的估计时间戳
    matched_timestamps = timestamps_estimated[estimated_indices]

    # 减去第一个关联时间戳，让时间从 0 秒开始
    elapsed_seconds = matched_timestamps - matched_timestamps[0]

    fig, ax = plt.subplots()

    # 横轴为相对时间，纵轴为 errors
    ax.plot(elapsed_seconds,
            errors)

    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Position error (m)")
    ax.set_title("Position error after rigid alignment")
    ax.grid(True)

    fig.savefig(error_plot_path, dpi=150)
    plt.close(fig)

    print("误差图已保存：", error_plot_path)

    # 找到最大误差所在的索引
    max_error_index = np.argmax(errors)

    print("最大位置误差（米）：", errors[max_error_index])
    print("出现时间（秒）：", elapsed_seconds[max_error_index])
    print("位置误差中位数（米）：", np.median(errors))

    report = {
        "estimated_file": str(trajectory_path_estimated),
        "groundtruth_file": str(trajectory_path_truth),
        "association": "nearest timestamp, no interpolation",
        "time_offset_s": 0.0,
        "max_time_difference_s": max_time_difference,
        "estimated_count": len(timestamps_estimated),
        "matched_count": len(matches),
        "alignment": "rotation and translation, no scaling",
        "ate_rmse_m": float(ate_rmse),
        "position_error_median_m": np.median(errors),
        "position_error_max_m": errors[max_error_index],
        "max_error_elapsed_s": elapsed_seconds[max_error_index],
    }

    with report_path.open("w", encoding="utf-8") as file:
        json.dump(report, file, indent=2, ensure_ascii=False, allow_nan=False)

    print("报告已保存：", report_path)

if __name__ == "__main__":
    main()
