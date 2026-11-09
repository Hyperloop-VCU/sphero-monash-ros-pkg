import math
import time
from typing import Optional, Tuple, Dict, Any

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from sphero_env.logging_utils import CsvStepLogger


class SpheroEnv(gym.Env):
    """
    2D kinematic point-mass environment with:

      - Configurable physical dimensions (meters or arbitrary units)
      - Configurable goal position
      - Binary occupancy grid for obstacles (for collision, mapping, etc.)
      - Ground-truth state with process noise in the dynamics
      - Separate odometry (odom) state using noise-free integration of commands
      - Observations derived from noisy odom state
      - Optional rendering of estimated belief (mean + covariance)

    Ground truth state (internal):
        state_true = [x, y, vx, vy]   (world frame)

    Odometry state (internal):
        state_odom = [x_o, y_o, vx_o, vy_o]
      - Integrated using the *commanded* velocity and heading with no process noise.
      - This naturally drifts away from state_true because state_true uses
        noisy dynamics.

    Observation (what the agent sees):
        obs = [x_meas, y_meas, vx_meas, vy_meas]
      where:
        - position and velocity are the odom state plus zero-mean measurement noise.

    Action:
        [v, theta] velocity (m/s) and heading (radians) command.

    Dynamics:
        - True dynamics are perturbed by process noise on acceleration.
        - Odom dynamics are the same but without process noise (ideal sensors).
        - This structure is suitable for teaching both RL and SLAM / state estimation.

    Occupancy grid:
        - occupancy_grid[h, w] ∈ {0,1}, 1 = obstacle.
        - grid_resolution in meters per cell.
        - World origin (0,0) at the center of the grid.

    Reward:
        - Negative Euclidean distance from *true* position to goal.
        - Optional collision penalty when hitting obstacles (true state).

        Termination:
                - Collision with obstacle (true state)
                - Out of world bounds (true state)
                - Time limit (max_steps -> truncated)
            Goal reach is reported in info["reached_goal"] but does not terminate.

    Rendering:
        - No in-process UI renderer is used.
        - This backend now supports logging/state APIs for ROS wrappers.
    """

    metadata = {"render_modes": ["human"], "render_fps": 60}

    def __init__(
        self,
        # Time & dynamics
        dt: float = 0.05,
        max_steps: int = 5000,
        action_limit: float = 0.1,
        vel_limit: float = 0.5,
        dynamics: Optional[None] = None,

        max_turn_rate: float = 2.0,          # rad/s
        heading_tolerance: float = 0.05,     # rad; must be within this to translate
        speed_scale: float = 1.0,              # scale factor for converting action speed to true speed

        # World dimensions (meters)
        world_width: float = 2.0,
        world_height: float = 2.0,

        # Goal configuration (meters in world frame)
        goal_pos: Tuple[float, float] = (0.0, 0.0),
        goal_tolerance: float = 0.2,

        # Occupancy grid (optional)
        occupancy_grid: Optional[np.ndarray] = None,
        grid_resolution: float = 0.1,  # meters per cell

        # Observation noise (std dev)
        obs_noise_std_pos: float = 0.05,
        obs_noise_std_vel: float = 0.05,

        # Logging
        csv_path: Optional[str] = None,

        # Dynamics / process noise on TRUE dynamics
        process_noise_std_heading: float = 0.1,   # heading noise std (radians)
        process_noise_std_speed: float = 0.1,   # speed noise std (m/s)

        # Rendering
        render_mode: Optional[str] = "human",
        window_size: Tuple[int, int] = (800, 800),
        collision_penalty: float = 1.0,

    ):
        super().__init__()

        # Dynamics params
        self.dt = dt
        self.max_steps = max_steps
        self.action_limit = action_limit
        self.vel_limit = vel_limit
        self.speed_scale = speed_scale

        self.max_turn_rate = max_turn_rate
        self.heading_tolerance = heading_tolerance

        # World geometry
        self.world_width = world_width
        self.world_height = world_height
        self.x_min = -world_width / 2.0
        self.x_max = world_width / 2.0
        self.y_min = -world_height / 2.0
        self.y_max = world_height / 2.0

        self.dynamics = dynamics

        # Goal
        self.goal_pos = np.array(goal_pos, dtype=np.float32)
        self.goal_tolerance = goal_tolerance
        self.current_setpoint = self.goal_pos.copy()
        self._use_custom_setpoint = False

        # Occupancy grid
        if occupancy_grid is not None:
            self.occupancy_grid = (occupancy_grid > 0).astype(np.uint8)
        else:
            self.occupancy_grid = None
        self.grid_resolution = grid_resolution

        # Noise parameters
        self.obs_noise_std_pos = obs_noise_std_pos
        self.obs_noise_std_vel = obs_noise_std_vel
        self.process_noise_std_heading = process_noise_std_heading
        self.process_noise_std_speed = process_noise_std_speed

        # Logging/rendering settings
        self.csv_path = csv_path
        self.last_command = None  # (heading_cmd, speed_cmd, timestamp)

        self.collision_penalty = collision_penalty

        # Internal states
        self.state_true: Optional[np.ndarray] = None   # [x, y, heading, speed]
        self.state_odom: Optional[np.ndarray] = None   # [x_o, y_o, heading_o, speed_o]
        self.last_obs: Optional[np.ndarray] = None
        self.step_count = 0
        self.latest_est_mean: Optional[np.ndarray] = None
        self.latest_est_cov: Optional[np.ndarray] = None

        # Collision flag carried into observation (0 or 1)
        self.last_collision_flag: float = 0.0

        _ = render_mode
        _ = window_size
        self.vis = CsvStepLogger(goal_pos=self.goal_pos, csv_path=self.csv_path)

        # ---- Gym spaces ----

        obs_low = np.array(
            [self.x_min, self.y_min, -np.pi, -self.vel_limit, 0.0],
            dtype=np.float32,
        )
        obs_high = np.array(
            [self.x_max, self.y_max, np.pi, self.vel_limit, 1.0],
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            dtype=np.float32,
        )


        # Action space: [v (m/s), theta (radians)]
        # v in [-vel_limit, vel_limit], theta in [-pi, pi]
        self.action_space = spaces.Box(
            low=np.array([-self.vel_limit, -np.pi], dtype=np.float32),
            high=np.array([self.vel_limit, np.pi], dtype=np.float32),
            dtype=np.float32,
        )

    def start_logging(self):
        self.vis.start_logging()

    def stop_logging(self):
        self.vis.stop_logging()

    def set_log_path(self, csv_path: Optional[str]):
        """Set/override CSV log path for the internal logger."""
        self.vis.csv_path = csv_path

    def set_current_setpoint(self, setpoint_xy: Optional[np.ndarray]):
        """Set the current position setpoint used for logging."""
        if setpoint_xy is None:
            self._use_custom_setpoint = False
            self.current_setpoint = self.goal_pos.copy()
            return
        setpoint_arr = np.asarray(setpoint_xy, dtype=np.float32).reshape(-1)
        if setpoint_arr.shape[0] < 2:
            raise ValueError("setpoint_xy must contain at least 2 values [x, y].")
        self.current_setpoint = setpoint_arr[:2].copy()
        self._use_custom_setpoint = True

    def update_estimate(
        self,
        est_mean: Optional[np.ndarray] = None,
        est_cov: Optional[np.ndarray] = None,
    ):
        """Update estimator belief used for rendering and step-wise logging."""
        if est_mean is None or est_cov is None:
            return
        self.vis.set_belief(est_mean, est_cov)
        self.latest_est_mean = np.asarray(est_mean, dtype=np.float32).copy()
        self.latest_est_cov = np.asarray(est_cov, dtype=np.float32).copy()

    def update_visualization(
        self,
        action: Optional[np.ndarray] = None,
        est_mean: Optional[np.ndarray] = None,
        est_cov: Optional[np.ndarray] = None,
    ):
        """Update estimator belief wrapper."""
        _ = action
        self.update_estimate(est_mean=est_mean, est_cov=est_cov)

    def log_estimator_metrics(
        self,
        P: np.ndarray,
        gt_pos: np.ndarray,
        est_pos: np.ndarray,
    ):
        """Backward-compatible EKF logging wrapper."""
        gt_pos = np.asarray(gt_pos, dtype=np.float32).reshape(-1)
        est_pos = np.asarray(est_pos, dtype=np.float32).reshape(-1)
        if gt_pos.shape[0] != 2 or est_pos.shape[0] != 2:
            raise ValueError("gt_pos and est_pos must have shape (2,)")

        gt_state = np.array([gt_pos[0], gt_pos[1], np.nan, np.nan], dtype=np.float32)
        est_state = np.array([est_pos[0], est_pos[1]], dtype=np.float32)
        self.vis.record(
            gt_state=gt_state,
            odom_state=None,
            action=None,
            est_state=est_state,
            est_cov=P,
            setpoint=self.current_setpoint,
        )

    # ---------------- Core API ---------------- #

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ):
        """Reset the environment to initial state.

        Args:
            seed: Random seed.
            options: Additional options.

        Returns:
            Initial observation and info.
        """
        super().reset(seed=seed)

        rng = self.np_random

        opts = options or {}
        init_pose = opts.get("init_pose")

        if init_pose is not None:
            init_arr = np.asarray(init_pose, dtype=np.float32).reshape(-1)
            if init_arr.shape[0] < 2:
                raise ValueError("options['init_pose'] must contain at least [x, y].")
            x0 = float(init_arr[0])
            y0 = float(init_arr[1])
            heading0 = (
                self._wrap_angle(float(init_arr[2]))
                if init_arr.shape[0] >= 3
                else float(rng.uniform(-np.pi, np.pi))
            )
            if self._is_collision(x0, y0):
                raise ValueError("options['init_pose'] is inside an obstacle.")
        else:
            # Prefer a random free spawn if a map is present.
            if self.occupancy_grid is not None:
                x0 = 0.0
                y0 = 0.0
                if self._is_collision(x0, y0):
                    free_cells = np.argwhere(self.occupancy_grid == 0)
                    if free_cells.size == 0:
                        raise RuntimeError("occupancy_grid has no free cells for reset spawn.")
                    i, j = free_cells[rng.integers(0, len(free_cells))]
                    h, w = self.occupancy_grid.shape
                    x0 = (float(j) + 0.5 - (w / 2.0)) * self.grid_resolution
                    y0 = ((h / 2.0) - (float(i) + 0.5)) * self.grid_resolution
            else:
                x0, y0 = 0.0, 0.0

            heading0 = float(rng.uniform(-np.pi, np.pi))

        speed0 = 0.0

        # No collision at reset
        self.last_collision_flag = 0.0

        # True and odom start the same
        self.state_true = np.array([x0, y0, heading0, speed0], dtype=np.float32)
        self.state_odom = self.state_true.copy()
        self.step_count = 0
        self.current_setpoint = self.goal_pos.copy()
        self._use_custom_setpoint = False
        self.latest_est_mean = None
        self.latest_est_cov = None

        # Clear visualiser state.
        self.vis.reset()
        self.vis.set_goal(self.goal_pos)

        obs = self._get_observation()
        self.last_obs = obs.copy()

        info = {
            "state_true": self.state_true.copy(),
            "state_odom": self.state_odom.copy(),
        }

        return obs, info

    def emergency_stop(self):
        """
        Immediately stop the robot (set speed to zero) without changing position or heading.

        This is a convenience method for students to call when they want to stop the robot
        without resetting the episode. It directly modifies the internal states.
        """
        if self.state_true is not None:
            self.state_true[3] = 0.0  # set true speed to zero
        if self.state_odom is not None:
            self.state_odom[3] = 0.0   # set odom speed to zero

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + np.pi) % (2.0 * np.pi) - np.pi
    
    def _base_dynamics(self, state, action):
        """
        Compute the next state given current state and action using the base dynamics without noise.
        This can be overridden by students to improve this.
        """

        x, y, heading, speed = state
        speed_cmd, heading_cmd = float(action[0]), float(action[1])

        desired_heading_true = self._wrap_angle(heading_cmd)
        desired_speed_true = self.speed_scale * np.clip(speed_cmd, -self.vel_limit, self.vel_limit)

        # Rotate first, limited turn rate
        heading_error_true = self._wrap_angle(desired_heading_true - heading)
        max_turn = self.max_turn_rate * self.dt
        heading_step_true = np.clip(heading_error_true, -max_turn, max_turn)
        heading_new = self._wrap_angle(heading + heading_step_true)

        # Only move if sufficiently aligned
        aligned_true = abs(self._wrap_angle(desired_heading_true - heading_new)) <= self.heading_tolerance
        speed_new = desired_speed_true if aligned_true else 0.0

        # 0 rad -> +y, pi/2 -> +x
        dx_true = speed_new * np.sin(heading_new) * self.dt
        dy_true = speed_new * np.cos(heading_new) * self.dt

        x_candidate = x + dx_true
        y_candidate = y + dy_true
        
        return np.array([x_candidate, y_candidate, heading_new, speed_new], dtype=np.float32)

    def step(self, action: np.ndarray):
        """Execute one step in the environment.

        Args:
            action: Action array [speed, heading].

        Returns:
            Observation, reward, terminated, truncated, info.
        """
        assert self.state_true is not None and self.state_odom is not None, \
            "Call reset() before step()."

        rng = self.np_random

        # By default, logged setpoint tracks goal_pos. Wrappers/controllers can
        # override this per-step via set_current_setpoint().
        if not self._use_custom_setpoint:
            self.current_setpoint = np.asarray(self.goal_pos, dtype=np.float32).copy()

        # Clip action
        action = np.clip(action, self.action_space.low, self.action_space.high)
        speed_cmd, heading_cmd = float(action[0]), float(action[1])
        heading_cmd = self._wrap_angle(heading_cmd)
        self.last_command = (heading_cmd, speed_cmd, time.time())

        moving_cmd = abs(speed_cmd) > 1e-6

        heading_noise = rng.normal(0.0, self.process_noise_std_heading)
        speed_noise = rng.normal(0.0, self.process_noise_std_speed) if moving_cmd else 0.0

        x, y, _, _ = self.state_true

        x_o, y_o, _, _ = self.state_odom

        noisy_action = action + np.array([speed_noise, heading_noise], dtype=np.float32)

        if self.dynamics is not None:
            # Ground truth is clean; odom carries process noise.
            state = self.dynamics(self.state_true, action)
            state_odom = self.dynamics(self.state_odom, noisy_action)
        else:
            # Same convention for default dynamics.
            state = self._base_dynamics(self.state_true, action)
            state_odom = self._base_dynamics(self.state_odom, noisy_action)

         # Unpack states
        x_candidate, y_candidate, heading_new, speed_new = state
        x_o_new, y_o_new, heading_o_new, speed_o_new = state_odom

        # Flags
        collision = False
        reached_goal = False
        out_of_bounds = False

        # =========================
        # Collision handling (true state + odom)
        # =========================
        if self._is_collision(x_candidate, y_candidate):
            collision = True
            x_new, y_new = self._resolve_collision(x, y, x_candidate, y_candidate)
            speed_new = 0.0
            # Odom is also contact-aware: back off and zero velocity
            x_o_new, y_o_new = self._resolve_collision(x_o, y_o, x_o_new, y_o_new)
            speed_o_new = 0.0
        else:
            x_new, y_new = x_candidate, y_candidate

        # Update states
        self.state_true = np.array([x_new, y_new, heading_new, speed_new], dtype=np.float32)
        self.state_odom = np.array([x_o_new, y_o_new, heading_o_new, speed_o_new], dtype=np.float32)

        self.step_count += 1

        # Termination
        terminated = False
        truncated = False

        if not (self.x_min <= x_new <= self.x_max and self.y_min <= y_new <= self.y_max):
            out_of_bounds = True

        dist_to_goal = float(
            math.hypot(x_new - self.goal_pos[0], y_new - self.goal_pos[1])
        )
        if dist_to_goal <= self.goal_tolerance:
            reached_goal = True

        if self.step_count >= self.max_steps:
            truncated = True

        reward = -dist_to_goal
        if collision:
            reward -= self.collision_penalty

        self.last_collision_flag = 1.0 if collision else 0.0

        obs = self._get_observation()
        self.last_obs = obs.copy()

        info: Dict[str, Any] = {
            "state_true": self.state_true.copy(),
            "state_odom": self.state_odom.copy(),
            "dist_to_goal": dist_to_goal,
            "collision": collision,
            "reached_goal": reached_goal,
            "out_of_bounds": out_of_bounds,
            "speed_cmd": speed_cmd,
            "heading_cmd": heading_cmd,
            "setpoint_xy": self.current_setpoint.copy(),
        }

        # Log each control step once; estimator updates are handled separately.
        self.vis.record(
            gt_state=self.state_true,
            odom_state=self.state_odom,
            action=np.array([speed_cmd, heading_cmd], dtype=np.float32),
            est_state=self.latest_est_mean,
            est_cov=self.latest_est_cov,
            setpoint=self.current_setpoint,
        )

        return obs, reward, terminated, truncated, info
    
    # --------------- Observation + belief --------------- #

    def _get_observation(self) -> np.ndarray:
        """
        Construct the observation based on the odometry state plus
        measurement noise. This is what the agent gets each step.
        """
        assert self.state_odom is not None
        rng = self.np_random

        x_o, y_o, heading_o, speed_o = self.state_odom

        x_meas = x_o + rng.normal(0.0, self.obs_noise_std_pos)
        y_meas = y_o + rng.normal(0.0, self.obs_noise_std_pos)
        heading_meas = self._wrap_angle(
            heading_o + rng.normal(0.0, self.obs_noise_std_vel)
        )
        moving_odom = abs(speed_o) > 1e-6
        speed_meas_noise = rng.normal(0.0, self.obs_noise_std_vel) if moving_odom else 0.0
        speed_meas = speed_o + speed_meas_noise

        obs = np.array(
            [x_meas, y_meas, heading_meas, speed_meas, self.last_collision_flag],
            dtype=np.float32,
        )
        obs = np.clip(obs, self.observation_space.low, self.observation_space.high)
        return obs

    def get_true_state(self) -> np.ndarray:
        """
        Return a copy of the ground-truth state [x, y, heading, speed].
        """
        assert self.state_true is not None
        return self.state_true.copy()

    def get_odom_state(self) -> np.ndarray:
        """
        Return a copy of the odometry state [x_o, y_o, heading_o, speed_o].
        """
        assert self.state_odom is not None
        return self.state_odom.copy()
    
    def set_belief(self, mean: np.ndarray, cov: np.ndarray):
        """
        Allow students to set their current belief (e.g., from EKF/UKF).

        mean:
            - shape (2,) or (4,) [x, y] or [x, y, vx, vy]
        cov:
            - shape (2, 2) or (4, 4)
              If (4,4), position covariance is cov[0:2, 0:2].

        This is only used for rendering an uncertainty ellipse; it does not
        affect the dynamics or reward.
        """
        mean = np.asarray(mean, dtype=np.float32)
        cov = np.asarray(cov, dtype=np.float32)

        if mean.shape not in [(2,), (4,)]:
            raise ValueError("mean must have shape (2,) or (4,).")
        if cov.shape not in [(2, 2), (4, 4)]:
            raise ValueError("cov must have shape (2,2) or (4,4).")

        self.vis.set_belief(mean, cov)

    # --------------- Occupancy / collision --------------- #

    def _is_collision(self, x: float, y: float) -> bool:
        """
        Check if world coordinates (x,y) lie in an obstacle cell.
        """
        if self.occupancy_grid is None:
            return False

        h, w = self.occupancy_grid.shape

        # World origin (0,0) at grid center
        cx = w / 2.0
        cy = h / 2.0

        # World -> grid indices
        j = int(np.floor(x / self.grid_resolution + cx))
        i = int(np.floor(-y / self.grid_resolution + cy))

        if i < 0 or i >= h or j < 0 or j >= w:
            # Outside grid: treat as free, bounds handle termination separately
            return False

        return self.occupancy_grid[i, j] == 1
    
    def _resolve_collision(self, x_old: float, y_old: float,
                       x_new: float, y_new: float,
                       n_substeps: int = 10) -> tuple[float, float]:
        """
        Given a motion from (x_old, y_old) to (x_new, y_new) that ends in collision,
        backtrack along the segment to find the last collision-free point.

        We linearly interpolate in n_substeps. The result is approximate but
        sufficient for teaching.
        """
        # If even the old point is in collision, just return old
        if self._is_collision(x_old, y_old):
            return x_old, y_old

        # Walk from old -> new, stop at first colliding substep,
        # and return the previous (free) point.
        for k in range(1, n_substeps + 1):
            t = k / n_substeps
            x_step = x_old + t * (x_new - x_old)
            y_step = y_old + t * (y_new - y_old)
            if self._is_collision(x_step, y_step):
                # previous free point
                t_prev = (k - 1) / n_substeps
                x_free = x_old + t_prev * (x_new - x_old)
                y_free = y_old + t_prev * (y_new - y_old)
                return x_free, y_free

        # Fallback: if none of the substeps were in collision, just return new
        return x_new, y_new

    # --------------- Rendering --------------- #

    def render(self):
        """No-op render hook maintained for interface compatibility."""
        self.vis.render(self.state_true, self.state_odom)

    def close(self):
        """Close the environment and clean up resources."""
        self.vis.close()