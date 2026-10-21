#!/usr/bin/env python3
"""Evaluate position ATE after timestamp association and a rigid, unscaled alignment."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Evaluation:
    indices: np.ndarray
    aligned_positions: np.ndarray
    truth_positions: np.ndarray
    elapsed_seconds: np.ndarray
    errors: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray


def load_trajectory(path):
    """Load eight-column TUM records; time must be finite and strictly increasing."""
    trajectory = np.loadtxt(path, ndmin=2)
    if trajectory.size == 0:
        raise ValueError(f"{path}：轨迹文件没有数值记录。")
    if trajectory.shape[1] != 8:
        raise ValueError(f"{path}：轨迹应为 8 列，实际为 {trajectory.shape[1]} 列。")
    if not np.isfinite(trajectory).all():
        raise ValueError(f"{path}：轨迹包含 NaN 或 Inf。")
    if not np.all(np.diff(trajectory[:, 0]) > 0):
        raise ValueError(f"{path}：时间戳必须严格递增，不能重复或倒序。")
    return trajectory


def associate_timestamps(estimated, truth, max_difference):
    """Nearest truth record for each estimate, without interpolation or time offset.

    This deliberately retains the original tool's policy: truth records may be
    reused. It is not a globally optimal or one-to-one matching algorithm.
    """
    if not np.isfinite(max_difference) or max_difference < 0:
        raise ValueError("--max-difference 必须是有限的非负秒数")
    if len(truth) == 0:
        raise ValueError("真值时间戳为空。")
    matches = []
    for index, timestamp in enumerate(estimated):
        differences = np.abs(truth - timestamp)
        nearest = int(np.argmin(differences))
        if differences[nearest] <= max_difference:
            matches.append((index, nearest))
    return np.asarray(matches, dtype=int).reshape(-1, 2)


def align_positions(estimated, truth):
    """Find a proper rotation and translation mapping estimated positions to truth."""
    if estimated.shape != truth.shape or estimated.ndim != 2 or estimated.shape[1] != 3:
        raise ValueError("对齐位置必须是形状相同的 N×3 数组。")
    if len(estimated) < 3 or not np.isfinite(estimated).all() or not np.isfinite(truth).all():
        raise ValueError("对齐至少需要三对有限位置。")
    estimated_center, truth_center = estimated.mean(axis=0), truth.mean(axis=0)
    centered_estimated, centered_truth = estimated - estimated_center, truth - truth_center
    if np.linalg.matrix_rank(centered_estimated) < 2 or np.linalg.matrix_rank(centered_truth) < 2:
        raise ValueError("关联位置退化：无法从重合或共线点唯一确定三维旋转。")
    U, _, Vt = np.linalg.svd(centered_estimated.T @ centered_truth)
    correction = np.eye(3)
    if np.linalg.det(Vt.T @ U.T) < 0:
        correction[2, 2] = -1  # Reflection is not a valid camera rotation.
    rotation = Vt.T @ correction @ U.T
    translation = truth_center - rotation @ estimated_center
    return estimated @ rotation.T + translation, rotation, translation


def evaluate_trajectories(estimated, truth, max_difference):
    indices = associate_timestamps(estimated[:, 0], truth[:, 0], max_difference)
    if len(indices) < 3:
        raise ValueError(f"只有 {len(indices)} 对时间关联，无法进行当前三维刚体对齐。")
    matched_estimated = estimated[indices[:, 0], 1:4]
    matched_truth = truth[indices[:, 1], 1:4]
    aligned, rotation, translation = align_positions(matched_estimated, matched_truth)
    timestamps = estimated[indices[:, 0], 0]
    return Evaluation(indices, aligned, matched_truth, timestamps - timestamps[0],
                      np.linalg.norm(aligned - matched_truth, axis=1), rotation, translation)


def make_report(result, estimated_path, truth_path, estimated_count, max_difference):
    errors = result.errors
    return {
        "estimated_file": str(estimated_path),
        "groundtruth_file": str(truth_path),
        "association": "nearest timestamp, no interpolation",
        "time_offset_s": 0.0,
        "max_time_difference_s": max_difference,
        "estimated_count": estimated_count,
        "matched_count": len(result.indices),
        "alignment": "rotation and translation, no scaling",
        "ate_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "position_error_median_m": float(np.median(errors)),
        "position_error_max_m": float(np.max(errors)),
        "max_error_elapsed_s": float(result.elapsed_seconds[np.argmax(errors)]),
    }


def save_plots(result, output_dir):
    """Save the X-Z projection and the full 3-D position error over elapsed time."""
    fig, ax = plt.subplots()
    ax.plot(result.truth_positions[:, 0], result.truth_positions[:, 2], label="Ground truth")
    ax.plot(result.aligned_positions[:, 0], result.aligned_positions[:, 2],
            label="Aligned estimate", linestyle="--")
    ax.set(xlabel="X (m)", ylabel="Z (m)", title="Aligned trajectories: X-Z projection")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True)
    fig.savefig(output_dir / "aligned_xz.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(result.elapsed_seconds, result.errors)
    ax.set(xlabel="Elapsed time (s)", ylabel="Position error (m)",
           title="Position error after rigid alignment")
    ax.grid(True)
    fig.savefig(output_dir / "position_error.png", dpi=150)
    plt.close(fig)


def parse_arguments():
    parser = argparse.ArgumentParser(description="轨迹关联与位置 ATE 评估")
    parser.add_argument("--max-difference", type=float, default=0.02,
                        help="最大关联时间差，单位：秒，默认 0.02")
    parser.add_argument("--estimated", type=Path,
                        default=ROOT / "results/tum-fr1-xyz-gXqAVwwv/CameraTrajectory.txt")
    parser.add_argument("--groundtruth", type=Path,
                        default=ROOT / "data/rgbd_dataset_freiburg1_xyz/groundtruth.txt")
    parser.add_argument("--output-dir", type=Path, help="结果目录，默认使用估计轨迹所在目录")
    args = parser.parse_args()
    if not np.isfinite(args.max_difference) or args.max_difference < 0:
        parser.error("--max-difference 必须是有限的非负秒数")
    return args


def main():
    args = parse_arguments()
    estimated_path, truth_path = args.estimated.resolve(), args.groundtruth.resolve()
    try:
        estimated, truth = load_trajectory(estimated_path), load_trajectory(truth_path)
        result = evaluate_trajectories(estimated, truth, args.max_difference)
        output_dir = args.output_dir.resolve() if args.output_dir else estimated_path.parent
        output_dir.mkdir(parents=True, exist_ok=True)
        save_plots(result, output_dir)
        report = make_report(result, estimated_path, truth_path, len(estimated), args.max_difference)
        (output_dir / "evaluation.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, OSError) as error:
        raise SystemExit(str(error)) from error
    print("位置关联数量：", report["matched_count"])
    print("ATE RMSE（米）：", report["ate_rmse_m"])
    print("报告和图像已保存：", output_dir)


if __name__ == "__main__":
    main()
