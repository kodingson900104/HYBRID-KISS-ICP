# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import contextlib
import datetime
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
from pyquaternion import Quaternion

from kiss_icp.config import load_config, write_config
from kiss_icp.kiss_icp import KissICP
from kiss_icp.metrics import absolute_trajectory_error, sequence_error
from kiss_icp.tools.pipeline_results import PipelineResults
from kiss_icp.tools.progress_bar import get_progress_bar
from kiss_icp.tools.visualizer import Kissualizer, StubVisualizer


def _rmse_index(p: np.ndarray, q: np.ndarray) -> float:
    """Index-to-index RMSE between two Nx3 arrays."""
    if p.size == 0 or q.size == 0:
        return float("nan")
    n = min(p.shape[0], q.shape[0])
    if n == 0:
        return float("nan")
    d = p[:n] - q[:n]
    return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))


def _apply_T(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply 4x4 transform to Nx3 points."""
    R = T[:3, :3]
    t = T[:3, 3]
    return (pts @ R.T) + t


class OdometryPipeline:
    def __init__(
        self,
        dataset,
        config: Optional[Path] = None,
        visualize: bool = False,
        n_scans: int = -1,
        jump: int = 0,
    ):
        self._dataset = dataset
        self._n_scans = (
            len(self._dataset) - jump if n_scans == -1 else min(len(self._dataset) - jump, n_scans)
        )
        self._jump = jump
        self._first = jump
        self._last = self._jump + self._n_scans

        # Config and output dir
        self.config = load_config(config)
        self.results_dir = None

        # Pipeline
        self.odometry = KissICP(config=self.config)
        self.results = PipelineResults()
        self.times = np.zeros(self._n_scans)
        self.poses = np.zeros((self._n_scans, 4, 4))
        self.has_gt = hasattr(self._dataset, "gt_poses")
        self.gt_poses = self._dataset.gt_poses[self._first : self._last] if self.has_gt else None
        self.dataset_name = self._dataset.__class__.__name__
        self.dataset_sequence = (
            self._dataset.sequence_id
            if hasattr(self._dataset, "sequence_id")
            else os.path.basename(self._dataset.data_dir)
        )

        # Visualizer
        self.visualizer = Kissualizer() if visualize else StubVisualizer()
        self._vis_infos = {
            "max_range": self.config.data.max_range,
            "min_range": self.config.data.min_range,
        }
        if hasattr(self._dataset, "use_global_visualizer"):
            self.visualizer._global_view = self._dataset.use_global_visualizer

        # [NEW] store only TWO raw frames for index-RMSE when n_scans == 2
        self._idx_rmse_frame0_xyz = None
        self._idx_rmse_frame1_xyz = None

    # Public interface  ------
    def run(self):
        self._run_pipeline()

        # [NEW] compute index-to-index RMSE only for 2-frame runs
        self._run_index_rmse_if_two_frames()

        self._run_evaluation()
        self._create_output_dir()
        self._write_result_poses()
        self._write_gt_poses()
        self._write_cfg()
        self._write_log()
        return self.results

    # Private interface  ------
    def _run_pipeline(self):
        for idx in get_progress_bar(self._first, self._last):
            raw_frame, timestamps = self._dataset[idx]
            start_time = time.perf_counter_ns()
            source, keypoints = self.odometry.register_frame(raw_frame, timestamps)
            self.poses[idx - self._first] = self.odometry.last_pose
            self.times[idx - self._first] = time.perf_counter_ns() - start_time

            # [NEW] store raw xyz for 2-frame dataset only
            if self._n_scans == 2:
                xyz = np.asarray(raw_frame[:, :3], dtype=np.float64)
                if idx == self._first:
                    self._idx_rmse_frame0_xyz = xyz
                elif idx == self._first + 1:
                    self._idx_rmse_frame1_xyz = xyz

            # Udate visualizer
            self._vis_infos["FPS"] = int(np.floor(self._get_fps()))
            self.visualizer.update(
                source,
                keypoints,
                self.odometry.local_map,
                self.odometry.last_pose,
                self._vis_infos,
            )

    def _run_index_rmse_if_two_frames(self):
        # Only meaningful when exactly 2 frames were processed
        if self._n_scans != 2:
            return
        if self._idx_rmse_frame0_xyz is None or self._idx_rmse_frame1_xyz is None:
            return

        P0 = self._idx_rmse_frame0_xyz  # frame0 raw
        P1 = self._idx_rmse_frame1_xyz  # frame1 raw

        # raw before (frame1 vs frame0, index-to-index)
        rmse_before = _rmse_index(P1, P0)

        # relative transform from estimated poses:
        # T_0_1 maps points in frame1 -> frame0 (via poses)
        T_w0 = self.poses[0]
        T_w1 = self.poses[1]
        T_0_1 = np.linalg.inv(T_w0) @ T_w1

        P1_aligned = _apply_T(T_0_1, P1)
        rmse_after = _rmse_index(P1_aligned, P0)

        # Print + also append into results table (units=m)
        print("\n[Index-to-Index RMSE (RAW points)]")
        print(f"rmse_before_raw(frame1 vs frame0)           = {rmse_before:.6f} m")
        print(f"rmse_after_aligned(inv(T_w0)*T_w1*frame1)   = {rmse_after:.6f} m\n")

        # Put into result metrics too (nice for logs)
        self.results.append(desc="Index RMSE before (raw f1 vs f0)", units="m", value=rmse_before)
        self.results.append(desc="Index RMSE after (aligned f1->f0)", units="m", value=rmse_after)

    @staticmethod
    def save_poses_kitti_format(filename: str, poses: np.ndarray):
        def _to_kitti_format(poses: np.ndarray) -> np.ndarray:
            return poses[:, :3].reshape(-1, 12)

        np.savetxt(fname=f"{filename}_kitti.txt", X=_to_kitti_format(poses))

    @staticmethod
    def save_poses_tum_format(filename, poses, timestamps):
        def _to_tum_format(poses, timestamps):
            tum_data = np.zeros((len(poses), 8))
            with contextlib.suppress(ValueError):
                for idx in range(len(poses)):
                    tx, ty, tz = poses[idx, :3, -1].flatten()
                    qw, qx, qy, qz = Quaternion(matrix=poses[idx], atol=0.01).elements
                    tum_data[idx] = np.r_[float(timestamps[idx]), tx, ty, tz, qx, qy, qz, qw]
                tum_data.flatten()
                return tum_data.astype(np.float64)

        np.savetxt(fname=f"{filename}_tum.txt", X=_to_tum_format(poses, timestamps), fmt="%.4f")

    def _calibrate_poses(self, poses):
        return (
            self._dataset.apply_calibration(poses)
            if hasattr(self._dataset, "apply_calibration")
            else poses
        )

    def _get_frames_timestamps(self):
        return (
            self._dataset.get_frames_timestamps()
            if hasattr(self._dataset, "get_frames_timestamps")
            else np.arange(0, self._n_scans, 1.0)
        )

    def _save_poses(self, filename: str, poses, timestamps):
        np.save(filename, poses)
        self.save_poses_kitti_format(filename, poses)
        self.save_poses_tum_format(filename, poses, timestamps)

    def _write_result_poses(self):
        self._save_poses(
            filename=f"{self.results_dir}/{self.dataset_sequence}_poses",
            poses=self._calibrate_poses(self.poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _write_gt_poses(self):
        if not self.has_gt:
            return
        self._save_poses(
            filename=f"{self.results_dir}/{self._dataset.sequence_id}_gt",
            poses=self._calibrate_poses(self.gt_poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _get_fps(self):
        times_nozero = self.times[self.times != 0]
        total_time_s = np.sum(times_nozero) * 1e-9
        return float(times_nozero.shape[0] / total_time_s) if total_time_s > 0 else 0

    def _run_evaluation(self):
        # Run estimation metrics evaluation, only when GT data was provided
        if self.has_gt:
            avg_tra, avg_rot = sequence_error(self.gt_poses, self.poses)
            ate_rot, ate_trans = absolute_trajectory_error(self.gt_poses, self.poses)
            self.results.append(desc="Average Translation Error", units="%", value=avg_tra)
            self.results.append(desc="Average Rotational Error", units="deg/m", value=avg_rot)
            self.results.append(desc="Absolute Trajectory Error (ATE)", units="m", value=ate_trans)
            self.results.append(desc="Absolute Rotational Error (ARE)", units="rad", value=ate_rot)

        fps = self._get_fps()
        if fps > 0:
            avg_fps = float(fps)
            avg_ms  = float(1e3 / fps)

            self.results.append(desc="Average Frequency", units="Hz", value=avg_fps, trunc=True)
            self.results.append(desc="Average Runtime",   units="ms", value=avg_ms,  trunc=True)

    def _write_log(self):
        if not self.results.empty():
            self.results.log_to_file(
                f"{self.results_dir}/result_metrics.log",
                f"Results for {self.dataset_name} Sequence {self.dataset_sequence}",
            )

    def _write_cfg(self):
        write_config(self.config, os.path.join(self.results_dir, "config.yml"))

    @staticmethod
    def _get_results_dir(out_dir: str):
        def get_current_timestamp() -> str:
            return datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        results_dir = os.path.join(os.path.realpath(out_dir), get_current_timestamp())
        latest_dir = os.path.join(os.path.realpath(out_dir), "latest")
        os.makedirs(results_dir, exist_ok=True)
        os.unlink(latest_dir) if os.path.exists(latest_dir) or os.path.islink(latest_dir) else None
        os.symlink(results_dir, latest_dir)
        return results_dir

    def _create_output_dir(self):
        self.results_dir = self._get_results_dir(self.config.out_dir)