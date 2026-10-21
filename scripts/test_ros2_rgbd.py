#!/usr/bin/env python3
"""End-to-end checks against installed ROS 2 nodes; run with the system Python.

Requires a sourced ROS Humble environment and an already built workbench. Tests use
an isolated ROS_DOMAIN_ID and never terminate nodes started outside this process.
"""

import argparse
from array import array
from contextlib import contextmanager
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
ASSOCIATIONS = ROOT / "third_party/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt"
VOCABULARY = ROOT / "third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
SETTINGS = ROOT / "configs/tum_fr1_rgbd.yaml"
DATASET = ROOT / "data/rgbd_dataset_freiburg1_xyz"
RECEIVER = ROOT / "ros2_ws/install/rgbd_slam/lib/rgbd_slam/rgbd_slam_node"
PLAYER = ROOT / "ros2_ws/install/rgbd_dataset_player/lib/rgbd_dataset_player/player_node"


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def ros_args(parameters):
    result = ["--ros-args"]
    for name, value in parameters.items():
        if isinstance(value, bool):
            value = str(value).lower()
        result += ["-p", f"{name}:={value}"]
    return result


class Suite:
    def __init__(self, args):
        # Import after setting the domain, so the test does not join a user's graph.
        global rclpy, Image, PoseStamped, UInt64
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Image
        from std_msgs.msg import UInt64

        self.args = args
        self.output = args.output_dir.resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.records = []
        self.poses = []
        rclpy.init()
        self.node = rclpy.create_node("workbench_integration_test")
        self.subscription = self.node.create_subscription(PoseStamped, "/slam/pose", self.on_pose, 100)
        self.rgb = self.node.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth = self.node.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.finish = self.node.create_publisher(UInt64, "/dataset/playback_finished", 1)
        self.receiver_parameters = {"vocabulary_path": VOCABULARY, "settings_path": SETTINGS}
        self.player_parameters = {"dataset_root": DATASET, "association_path": ASSOCIATIONS,
                                  "max_frames": 1, "publish_period_ms": 10,
                                  "wait_for_subscribers": False}
        self.rows = [line.split() for line in ASSOCIATIONS.read_text().splitlines()
                     if line.strip() and not line.lstrip().startswith("#")]
        self.metadata = {
            "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            "git_status": subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True),
            "ros_domain_id": args.domain, "frames": args.frames,
            "publish_period_ms": args.period_ms,
            "dependencies": json.loads((ROOT / "dependencies.json").read_text()),
            "sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in [SETTINGS, ASSOCIATIONS,
                                    ROOT / "ros2_ws/src/rgbd_slam/src/rgbd_slam_node.cpp",
                                    ROOT / "ros2_ws/src/rgbd_dataset_player/src/player_node.cpp"]}}

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.poses.append({"stamp_ns": msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                           "frame_id": msg.header.frame_id, "position": [p.x, p.y, p.z],
                           "quaternion": [q.x, q.y, q.z, q.w]})

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.02, max(0, end - time.monotonic())))

    def until(self, condition, seconds):
        end = time.monotonic() + seconds
        while not condition():
            require(time.monotonic() < end, "test deadline exceeded")
            self.spin(0.02)

    def save(self):
        (self.output / "report.json").write_text(json.dumps(
            {"metadata": self.metadata, "tests": self.records}, indent=2))

    def case(self, name, function):
        started = time.monotonic()
        try:
            details = function() or {}
            record = {"name": name, "passed": True, **details}
        except Exception as error:
            record = {"name": name, "passed": False, "error": str(error)}
        record["seconds"] = round(time.monotonic() - started, 3)
        self.records.append(record)
        self.save()
        print(json.dumps(record), flush=True)
        self.spin(0.2)

    @staticmethod
    def cleanup(process):
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

    def command(self, name, executable, parameters, marker, rc=1):
        result = subprocess.run([str(executable)] + ros_args(parameters), cwd=self.output,
                                capture_output=True, text=True, timeout=12)
        text = result.stdout + result.stderr
        (self.output / f"{name}.log").write_text(text)
        require(result.returncode == rc, f"expected exit {rc}, got {result.returncode}")
        require(marker in text, f"missing log: {marker}")
        return {"exit_code": result.returncode}

    @contextmanager
    def receiver(self, name, timeout=1.0):
        path = self.output / f"{name}.log"
        self.poses.clear()
        with path.open("w") as stream:
            process = subprocess.Popen([str(RECEIVER)] + ros_args(
                {**self.receiver_parameters, "finish_timeout_sec": timeout}),
                cwd=self.output, stdout=stream, stderr=subprocess.STDOUT)
            try:
                def ready():
                    require(process.poll() is None, "receiver exited before ready; inspect its log")
                    return ("Vocabulary loaded!" in path.read_text() and
                            self.node.count_publishers("/slam/pose") > 0 and
                            all(p.get_subscription_count() for p in [self.rgb, self.depth, self.finish]))
                self.until(ready, 45)
                self.spin(0.2)
                yield process, path
            finally:
                self.cleanup(process)

    def wait_receiver(self, process, path, timeout=8):
        self.until(lambda: process.poll() is not None, timeout)
        self.spin(0.3)
        return path.read_text()

    @staticmethod
    def image(kind, index=0, tail=False, empty=False, malformed=False, mismatch=False, blank=False):
        msg = Image()
        msg.header.stamp.sec = 100
        msg.header.stamp.nanosec = index * 100_000_000 + (10_000_000 if tail and kind == "depth" else 0)
        msg.header.frame_id = "camera_color_optical_frame"
        msg.width = msg.height = 0 if empty else 1
        if blank:
            msg.width, msg.height = 640, 480
        if mismatch:
            msg.width += 1
        msg.encoding = "bgr8" if kind == "rgb" else "16UC1"
        msg.step = msg.width * (3 if kind == "rgb" else 2)
        msg.data = array("B", bytes(0 if malformed else msg.height * msg.step))
        return msg

    def fault(self, name, nrgb, ndepth, marker, repeat=False):
        with self.receiver(name) as (process, path):
            # Only one channel is sent; the synthetic 1x1 image never reaches TrackRGBD.
            for kind, count, pub in [("rgb", nrgb, self.rgb), ("depth", ndepth, self.depth)]:
                for i in range(count):
                    pub.publish(self.image(kind, i))
                    self.spin(0.1)
            self.spin(0.15)
            start = time.monotonic()
            self.finish.publish(UInt64(data=1))
            while process.poll() is None and time.monotonic() - start < 4:
                self.spin(0.15)
                if repeat:
                    self.finish.publish(UInt64(data=0))
            text = self.wait_receiver(process, path)
            require(process.returncode == 1 and marker in text, "wrong failure path or exit code")
            require(f"expected=1, RGB={nrgb}, depth={ndepth}" in text, "wrong frozen count")
            require(f"Summary: RGB={nrgb}, depth={ndepth}, pairs=0, tail_pairs=0" in text, "wrong summary")
            require("Shutdown" in text and not self.poses, "unexpected pose or missing shutdown")
            if repeat:
                require("Ignoring duplicate" in text, "duplicate notification was not exercised")
                waited = float(re.search(r"Timed out after ([\d.]+) s", text)[1])
                require(1 <= waited < 1.5, "duplicate notifications delayed the deadline")
            return {"exit_code": process.returncode}

    def invalid_pair(self, name, mode, tail):
        with self.receiver(name) as (process, path):
            self.rgb.publish(self.image("rgb", tail=tail, empty=mode == "empty",
                                        malformed=mode == "malformed"))
            self.spin(0.1)
            self.depth.publish(self.image("depth", tail=tail, mismatch=mode == "mismatch"))
            self.spin(0.2)
            if tail:
                require(process.poll() is None, "tail was processed before finish notification")
            if process.poll() is None:
                self.finish.publish(UInt64(data=1))
            text = self.wait_receiver(process, path)
            marker = "Image conversion failed" if mode == "malformed" else "Invalid RGB-D image shape or type"
            require(process.returncode == 1 and marker in text, "wrong image failure path")
            require("Summary: RGB=1, depth=1, pairs=0, tail_pairs=0" in text, "failed pair counted as success")
            require(("Processing one pending tail pair" in text) == tail, "wrong processing path")
            require(not self.poses, "invalid image produced a pose")
            return {"exit_code": process.returncode}

    def blank(self):
        with self.receiver("not_initialized") as (process, path):
            for i in range(5):
                self.rgb.publish(self.image("rgb", i, blank=True))
                self.depth.publish(self.image("depth", i, blank=True))
                self.spin(0.2)
            self.finish.publish(UInt64(data=5))
            text = self.wait_receiver(process, path)
            states = list(map(int, re.findall(r"Tracking state: (-?\d+)", text)))
            require(process.returncode == 0 and states == [1] * 5, "blank input did not stay uninitialized")
            require(not self.poses, "uninitialized tracking published a pose")
            return {"states": states, "pose_count": 0}

    def interrupt(self):
        with self.receiver("interrupt") as (process, path):
            process.send_signal(signal.SIGINT)
            text = self.wait_receiver(process, path)
            require(process.returncode == 0 and "Shutdown" in text, "Ctrl+C did not call core shutdown")
            return {"exit_code": process.returncode}

    def regression(self):
        count = min(self.args.frames, len(self.rows))
        with self.receiver("real_receiver", timeout=5.0) as (receiver, path):
            with (self.output / "real_player.log").open("w") as stream:
                player = subprocess.Popen([str(PLAYER)] + ros_args(
                    {**self.player_parameters, "wait_for_subscribers": True, "max_frames": count,
                     "publish_period_ms": self.args.period_ms}),
                    cwd=self.output, stdout=stream, stderr=subprocess.STDOUT)
                try:
                    self.until(lambda: receiver.poll() is not None and player.poll() is not None,
                               count * self.args.period_ms / 1000 + 40)
                    text = self.wait_receiver(receiver, path)
                    require(receiver.returncode == player.returncode == 0, "normal run failed")
                finally:
                    self.cleanup(player)
            (self.output / "poses.json").write_text(json.dumps(self.poses, indent=2))
            pairs = re.findall(r"Pair: rgb=(\d+\.\d+), depth=(\d+\.\d+)", text)
            expected = [(r[0] + "000", r[2] + "000") for r in self.rows[:count]]
            states = list(map(int, re.findall(r"Tracking state: (-?\d+)", text)))
            require(pairs == expected, "missing, repeated or reordered image pairs")
            require(len(states) == count, "state count differs from processed frames")
            expected_stamps = [int(a) * 10**9 + int(b) for state, (stamp, _) in zip(states, pairs)
                               if state == 2 for a, b in [stamp.split(".")]]
            stamps = [p["stamp_ns"] for p in self.poses]
            require(stamps == expected_stamps and bool(stamps), "poses do not match OK RGB frames")
            require(len(set(stamps)) == len(stamps), "duplicate pose stamps")
            require(all(p["frame_id"] == "map" for p in self.poses), "wrong pose frame")
            require(all(math.isfinite(v) for p in self.poses for v in p["position"] + p["quaternion"]), "non-finite pose")
            norm_error = max(abs(math.sqrt(sum(v*v for v in p["quaternion"])) - 1) for p in self.poses)
            require(norm_error < 1e-5, "non-unit quaternion")
            summary = re.search(r"Summary: RGB=(\d+), depth=(\d+), pairs=(\d+), tail_pairs=(\d+)", text)
            require(summary is not None and list(map(int, summary.groups()[:3])) == [count]*3, "wrong final counts")
            require(int(summary[4]) in (0, 1), "more than one explicit tail")
            require("[ERROR]" not in text, "receiver logged an error")
            return {"frames": count, "OK_frames": states.count(2), "pose_count": len(self.poses),
                    "tail_pairs": int(summary[4]), "max_quaternion_norm_error": norm_error}

    def run(self):
        if self.args.only_real:
            self.case("real_sequence", self.regression)
            return all(record["passed"] for record in self.records)
        # Parameter errors must fail before vocabulary loading or subscriptions.
        for name, value in [("zero", "0.0"), ("negative", "-1.0"), ("nan", ".nan"), ("inf", ".inf")]:
            parameter_file = self.output / f"{name}.yaml"
            parameter_file.write_text(f"/**:\n  ros__parameters:\n    finish_timeout_sec: {value}\n")
            def check(name=name, path=parameter_file):
                result = subprocess.run([str(RECEIVER), "--ros-args", "--params-file", str(path)],
                                        capture_output=True, text=True, timeout=10, cwd=self.output)
                text = result.stdout + result.stderr
                (self.output / f"parameter_{name}.log").write_text(text)
                require(result.returncode == 1 and "positive and finite" in text, "invalid timeout accepted")
            self.case("timeout_" + name, check)
        for name, overrides in [("missing_vocabulary", {"vocabulary_path": self.output / "absent"}),
                                ("settings_directory", {"settings_path": ROOT / "configs"})]:
            self.case(name, lambda name=name, overrides=overrides: self.command(
                name, RECEIVER, {**self.receiver_parameters, **overrides}, "Vocabulary or settings file does not exist"))
        for name, overrides in [("negative_max_frames", {"max_frames": -1}), ("zero_period", {"publish_period_ms": 0})]:
            self.case(name, lambda name=name, overrides=overrides: self.command(
                name, PLAYER, {**self.player_parameters, **overrides}, "max_frames must be >= 0"))
        for index, stamp in enumerate(["abc.123456", "1305031102.x75304", "1305031102.123", "2147483648.000000"]):
            association = self.output / f"bad_stamp_{index}.txt"
            row = list(self.rows[0]); row[0] = stamp
            association.write_text(" ".join(row) + "\n")
            self.case(f"bad_timestamp_{index}", lambda index=index, association=association: self.command(
                f"bad_stamp_{index}", PLAYER, {**self.player_parameters, "association_path": association}, "timestamp" if index < 3 else "Timestamp"))
        for name, contents, marker in [("empty_association", "# comment\n\n", "Association file is empty"),
                                       ("malformed_association", "only two\n", "Fields read error")]:
            association = self.output / f"{name}.txt"; association.write_text(contents)
            self.case(name, lambda name=name, association=association, marker=marker: self.command(
                name, PLAYER, {**self.player_parameters, "association_path": association}, marker))
        for kind in ["rgb", "depth"]:
            association = self.output / f"missing_{kind}_image.txt"
            row = list(self.rows[0]); row[1 if kind == "rgb" else 3] = f"{self.output}/absent.png"
            association.write_text(" ".join(row) + "\n")
            self.case(f"missing_{kind}_image", lambda kind=kind, association=association: self.command(
                f"missing_{kind}_image", PLAYER, {**self.player_parameters, "association_path": association},
                "Cannot read RGB image" if kind == "rgb" else "Cannot read DEPTH image"))
        # Whitespace and leading comments are accepted without changing the first frame.
        association = self.output / "comments.txt"
        association.write_text("  # comment\n\n" + " ".join(self.rows[0]) + "\n")
        self.case("leading_comments", lambda: self.command("leading_comments", PLAYER,
            {**self.player_parameters, "association_path": association}, "Playback finished: 1 pairs published", rc=0))
        for name, nrgb, ndepth, marker in [("missing_both", 0, 0, "Timed out after"),
            ("missing_rgb", 0, 1, "Timed out after"), ("missing_depth", 1, 0, "Timed out after"),
            ("excess_rgb", 2, 0, "Received too many images"), ("excess_depth", 0, 2, "Received too many images")]:
            self.case(name, lambda name=name, nrgb=nrgb, ndepth=ndepth, marker=marker: self.fault(name, nrgb, ndepth, marker))
        self.case("duplicate_finish", lambda: self.fault("duplicate_finish", 0, 0, "Timed out after", repeat=True))
        for tail in [False, True]:
            for mode in ["empty", "mismatch", "malformed"]:
                name = ("tail_" if tail else "sync_") + mode
                self.case(name, lambda name=name, mode=mode, tail=tail: self.invalid_pair(name, mode, tail))
        self.case("not_initialized", self.blank)
        self.case("interrupt", self.interrupt)
        self.case("real_sequence", self.regression)
        passed = sum(record["passed"] for record in self.records)
        print(f"Passed {passed}/{len(self.records)}; report: {self.output / 'report.json'}", flush=True)
        return passed == len(self.records)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=30, help="real input pairs; use 792 for the full sequence")
    parser.add_argument("--period-ms", type=int, default=200, help="fixed playback interval, not dataset time scaling")
    parser.add_argument("--domain", type=int, default=200, help="dedicated test ROS domain, 1..232")
    parser.add_argument("--only-real", action="store_true", help="run only the real-data topic regression")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results" / datetime.now().strftime("ros2-validation-%Y%m%d-%H%M%S"))
    args = parser.parse_args()
    if not (args.frames > 0 and args.period_ms > 0 and 1 <= args.domain <= 232):
        parser.error("frames and period must be positive; domain must be 1..232")
    for path in [RECEIVER, PLAYER, VOCABULARY, SETTINGS, ASSOCIATIONS]:
        if not path.is_file():
            parser.error(f"missing prerequisite: {path}")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOG_DIR"] = str(args.output_dir.resolve() / "roslogs")
    suite = Suite(args)
    try:
        return 0 if suite.run() else 1
    finally:
        suite.close()


if __name__ == "__main__":
    sys.exit(main())
