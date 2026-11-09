import csv
import os
import threading
from typing import Optional

import numpy as np


class CsvStepLogger:
    """Lightweight logger compatible with the former visualiser interface."""

    def __init__(
        self,
        goal_pos: np.ndarray,
        csv_path: Optional[str] = None,
    ):
        self.goal_pos = np.asarray(goal_pos, dtype=np.float32)
        self.csv_path = csv_path
        self._lock = threading.Lock()
        self._file = None
        self._writer = None
        self._header_written = False
        self._traj_x: list[float] = []
        self._traj_y: list[float] = []
        self._true_traj_x: list[float] = []
        self._true_traj_y: list[float] = []
        self._est_mean: Optional[np.ndarray] = None
        self._est_cov: Optional[np.ndarray] = None

    def start_logging(self):
        if not self.csv_path:
            return
        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        self._file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._header_written = False

    def stop_logging(self):
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None

    def set_goal(self, goal_pos: np.ndarray):
        self.goal_pos = np.asarray(goal_pos, dtype=np.float32)

    def set_belief(self, mean: np.ndarray, cov: np.ndarray):
        self._est_mean = np.asarray(mean, dtype=np.float32)
        self._est_cov = np.asarray(cov, dtype=np.float32)

    def reset(self):
        self._traj_x.clear()
        self._traj_y.clear()
        self._true_traj_x.clear()
        self._true_traj_y.clear()

    def record(
        self,
        gt_state: Optional[np.ndarray],
        odom_state: Optional[np.ndarray],
        action: Optional[np.ndarray],
        est_state: Optional[np.ndarray],
        est_cov: Optional[np.ndarray],
        setpoint: Optional[np.ndarray],
    ):
        with self._lock:
            if odom_state is not None:
                self._traj_x.append(float(odom_state[0]))
                self._traj_y.append(float(odom_state[1]))
            if gt_state is not None:
                self._true_traj_x.append(float(gt_state[0]))
                self._true_traj_y.append(float(gt_state[1]))

            if self._writer is None:
                return

            if not self._header_written:
                self._writer.writerow(
                    [
                        "est_x",
                        "est_y",
                        "odom_x",
                        "odom_y",
                        "gt_x",
                        "gt_y",
                        "cov_x",
                        "cov_y",
                        "cov_xy",
                        "cov_yx",
                        "heading_cmd",
                        "speed_cmd",
                        "heading",
                        "speed",
                        "setpoint_x",
                        "setpoint_y",
                    ]
                )
                self._header_written = True

            est_xy = np.asarray(est_state, dtype=np.float32).reshape(-1) if est_state is not None else np.array([np.nan, np.nan], dtype=np.float32)
            if est_xy.shape[0] < 2:
                est_xy = np.array([np.nan, np.nan], dtype=np.float32)

            cov = np.asarray(est_cov, dtype=np.float32) if est_cov is not None else np.full((2, 2), np.nan, dtype=np.float32)
            if cov.shape == (4, 4):
                cov = cov[:2, :2]
            if cov.shape != (2, 2):
                cov = np.full((2, 2), np.nan, dtype=np.float32)

            odom = np.asarray(odom_state, dtype=np.float32) if odom_state is not None else np.full(4, np.nan, dtype=np.float32)
            gt = np.asarray(gt_state, dtype=np.float32) if gt_state is not None else np.full(4, np.nan, dtype=np.float32)
            cmd = np.asarray(action, dtype=np.float32) if action is not None else np.array([np.nan, np.nan], dtype=np.float32)
            sp = np.asarray(setpoint, dtype=np.float32).reshape(-1) if setpoint is not None else np.array([np.nan, np.nan], dtype=np.float32)
            if sp.shape[0] < 2:
                sp = np.array([np.nan, np.nan], dtype=np.float32)

            self._writer.writerow(
                [
                    float(est_xy[0]),
                    float(est_xy[1]),
                    float(odom[0]),
                    float(odom[1]),
                    float(gt[0]),
                    float(gt[1]),
                    float(cov[0, 0]),
                    float(cov[1, 1]),
                    float(cov[0, 1]),
                    float(cov[1, 0]),
                    float(cmd[1]),
                    float(cmd[0]),
                    float(odom[2]),
                    float(odom[3]),
                    float(sp[0]),
                    float(sp[1]),
                ]
            )

    def get_traj(self):
        return list(self._traj_x), list(self._traj_y), None

    def render(self, _state_true=None, _state_odom=None):
        return

    def close(self):
        self.stop_logging()
