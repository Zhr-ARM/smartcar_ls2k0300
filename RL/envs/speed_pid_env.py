import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from RL.models.system_id import load_identification

try:
    import gymnasium as gym
    from gymnasium import spaces
except Exception:  # pragma: no cover
    gym = None

    class _SimpleBox:
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.low = np.array(low, dtype=dtype)
            self.high = np.array(high, dtype=dtype)
            self.shape = tuple(shape or self.low.shape)
            self.dtype = dtype

        def sample(self):
            return np.random.uniform(self.low, self.high).astype(self.dtype)

    class _Spaces:
        Box = _SimpleBox

    spaces = _Spaces()


PARAM_ORDER = [
    "left_kp",
    "left_ki",
    "left_feedforward_gain",
    "right_kp",
    "right_ki",
    "right_feedforward_gain"
]


@dataclass
class WheelRuntime:
    y_prev1: float = 0.0
    y_prev2: float = 0.0
    raw_window: Optional[deque] = None
    filtered_feedback: float = 0.0
    error_prev1: float = 0.0
    error_prev2: float = 0.0
    correction_last: float = 0.0
    duty_prev1: float = 0.0
    duty_prev2: float = 0.0


class SpeedPidEnv(gym.Env if gym else object):
    metadata = {"render_modes": []}

    def __init__(self, task_config_path: str, model_path: str, seed: int = 42):
        self.task_config_path = Path(task_config_path)
        self.model_path = Path(model_path)
        self.task = json.loads(self.task_config_path.read_text(encoding="utf-8"))
        self.model = load_identification(self.model_path)
        self.rng = np.random.default_rng(seed)
        self.training_cfg = self.task["training"]
        self.reward_cfg = self.task["reward"]
        self.bounds = self.task["parameter_bounds"]
        low = [self.bounds[key][0] for key in PARAM_ORDER]
        high = [self.bounds[key][1] for key in PARAM_ORDER]
        self.action_space = spaces.Box(low=np.array(low, dtype=np.float32), high=np.array(high, dtype=np.float32), dtype=np.float32)
        self.observation_space = spaces.Box(low=-1e6, high=1e6, shape=(22,), dtype=np.float32)
        self.max_episode_evals = int(self.training_cfg.get("max_episode_evals", 10))
        self.eval_count = 0
        self.last_metrics = {}
        self.last_action = np.array([self.task["initial_params"][key] for key in PARAM_ORDER], dtype=np.float32)
        self.current_sequence = []

    def _sample_sequence(self) -> List[Dict]:
        base = list(self.task["capture"]["step_sequence"])
        sampled = []
        prev_left = 0.0
        prev_right = 0.0
        for step in base:
            left = float(step["left_target"])
            right = float(step["right_target"])
            left += float(self.rng.uniform(-18.0, 18.0)) if left > 0 else 0.0
            right += float(self.rng.uniform(-18.0, 18.0)) if right > 0 else 0.0
            left = float(np.clip(left, max(prev_left - 220.0, 0.0), min(prev_left + 220.0, 620.0)))
            right = float(np.clip(right, max(prev_right - 220.0, 0.0), min(prev_right + 220.0, 620.0)))
            prev_left, prev_right = left, right
            sampled.append({
                "left_target": left,
                "right_target": right,
                "duration_ms": int(step["duration_ms"]),
                "tag": step.get("tag", "step")
            })
        return sampled

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.eval_count = 0
        self.current_sequence = self._sample_sequence()
        self.last_metrics = {
            "reward": 0.0,
            "main_error_sum": 0.0,
            "tail_error_sum": 0.0,
            "overshoot_sum": 0.0,
            "heat_penalty": 0.0
        }
        return self._observation(), {}

    def _clip_params(self, action: np.ndarray) -> Dict[str, float]:
        params = {}
        for idx, key in enumerate(PARAM_ORDER):
            lo, hi = self.bounds[key]
            params[key] = float(np.clip(float(action[idx]), lo, hi))
        return params

    def _new_wheel_runtime(self) -> WheelRuntime:
        cfg = self.task["controller_defaults"]
        return WheelRuntime(
            raw_window=deque(maxlen=max(1, int(cfg["feedback_average_window"])))
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _simulate_wheel(self, wheel: str, target_sequence: List[float], params: Dict[str, float]) -> Dict:
        ctrl = self.task["controller_defaults"]
        plant = self.model["wheels"][wheel]["plant"]
        runtime = self._new_wheel_runtime()

        prefix = f"{wheel}_"
        kp = params[f"{wheel}_kp"]
        ki = params[f"{wheel}_ki"]
        ff_gain = params[f"{wheel}_feedforward_gain"]
        ff_bias = float(ctrl[f"{wheel}_feedforward_bias"])

        feedbacks = []
        duties = []
        errors = []
        overshoot = 0.0
        heat_penalty = 0.0

        for target in target_sequence:
            error = float(target - runtime.filtered_feedback)
            p = kp * (error - runtime.error_prev1)
            i = ki * self._clamp(error, float(ctrl["integral_limit"]))
            d = 0.0
            correction = runtime.correction_last + p + i + d
            correction = self._clamp(correction, float(ctrl["correction_limit"]))
            delta = correction - runtime.correction_last
            if abs(delta) > float(ctrl["max_output_step"]):
                correction = runtime.correction_last + math.copysign(float(ctrl["max_output_step"]), delta)

            feedforward = ff_gain * target
            if abs(target) >= float(ctrl["feedforward_bias_threshold"]):
                feedforward += math.copysign(ff_bias, target if target != 0 else 1.0)

            decel_assist = 0.0
            if error < -float(ctrl["decel_error_threshold"]):
                decel_assist = self._clamp(
                    float(ctrl["decel_duty_gain"]) * abs(error),
                    float(ctrl["decel_duty_limit"])
                )
                decel_assist *= -1.0

            duty = feedforward + correction + decel_assist
            duty = self._clamp(duty, float(ctrl["duty_limit"]))

            y = (
                float(plant["a1"]) * runtime.y_prev1 +
                float(plant["a2"]) * runtime.y_prev2 +
                float(plant["b1"]) * runtime.duty_prev1 +
                float(plant["b2"]) * runtime.duty_prev2 +
                float(plant["bias"])
            )

            runtime.raw_window.append(y)
            avg_feedback = sum(runtime.raw_window) / max(1, len(runtime.raw_window))
            alpha = float(ctrl["feedback_low_pass_alpha"])
            runtime.filtered_feedback = alpha * avg_feedback + (1.0 - alpha) * runtime.filtered_feedback

            runtime.error_prev2 = runtime.error_prev1
            runtime.error_prev1 = error
            runtime.correction_last = correction
            runtime.y_prev2 = runtime.y_prev1
            runtime.y_prev1 = y
            runtime.duty_prev2 = runtime.duty_prev1
            runtime.duty_prev1 = duty

            feedbacks.append(float(runtime.filtered_feedback))
            duties.append(float(duty))
            errors.append(float(target - runtime.filtered_feedback))
            overshoot += max(0.0, runtime.filtered_feedback - target) if target >= 0.0 else 0.0
            heat_penalty += max(0.0, abs(duty) - 0.82 * float(ctrl["duty_limit"])) * 0.4
            if len(duties) >= 2:
                heat_penalty += abs(duties[-1] - duties[-2]) * 0.02

        return {
            "feedbacks": feedbacks,
            "duties": duties,
            "errors": errors,
            "overshoot": overshoot,
            "heat_penalty": heat_penalty
        }

    def _expand_targets(self, sequence: List[Dict]) -> Tuple[List[float], List[float], List[str]]:
        left_targets = []
        right_targets = []
        tags = []
        for step in sequence:
            ticks = max(1, int(round(float(step["duration_ms"]) / 5.0)))
            for _ in range(ticks):
                left_targets.append(float(step["left_target"]))
                right_targets.append(float(step["right_target"]))
                tags.append(str(step.get("tag", "step")))
        return left_targets, right_targets, tags

    def _evaluate(self, params: Dict[str, float]) -> Dict:
        left_targets, right_targets, _ = self._expand_targets(self.current_sequence)
        left = self._simulate_wheel("left", left_targets, params)
        right = self._simulate_wheel("right", right_targets, params)

        left_abs = np.abs(np.asarray(left["errors"], dtype=np.float64))
        right_abs = np.abs(np.asarray(right["errors"], dtype=np.float64))
        first2 = float(np.sum(left_abs[:2]) + np.sum(right_abs[:2]))
        first20 = float(np.sum(left_abs[:20]) + np.sum(right_abs[:20]))
        overshoot = float(left["overshoot"] + right["overshoot"])
        heat = float(left["heat_penalty"] + right["heat_penalty"])
        reward = (
            -first2
            - float(self.reward_cfg["tail_weight"]) * first20
            - float(self.reward_cfg["overshoot_weight"]) * overshoot
            - float(self.reward_cfg["heat_weight"]) * heat
        )
        return {
            "reward": reward,
            "main_error_sum": first2,
            "tail_error_sum": first20,
            "overshoot_sum": overshoot,
            "heat_penalty": heat,
            "left": left,
            "right": right,
            "sequence": self.current_sequence
        }

    def _observation(self) -> np.ndarray:
        metrics = self.last_metrics
        seq = self.current_sequence or self._sample_sequence()
        current = seq[min(len(seq) - 1, 1)] if seq else {"left_target": 0.0, "right_target": 0.0}
        obs = np.asarray([
            float(current.get("left_target", 0.0)),
            float(current.get("right_target", 0.0)),
            float(metrics.get("reward", 0.0)),
            float(metrics.get("main_error_sum", 0.0)),
            float(metrics.get("tail_error_sum", 0.0)),
            float(metrics.get("overshoot_sum", 0.0)),
            float(metrics.get("heat_penalty", 0.0)),
            *self.last_action.tolist(),
            float(self.eval_count),
            float(self.max_episode_evals),
            float(self.model["wheels"]["left"]["plant"]["mse"]),
            float(self.model["wheels"]["right"]["plant"]["mse"]),
            float(self.task["controller_defaults"]["duty_limit"]),
            float(self.task["controller_defaults"]["max_output_step"]),
            float(self.task["controller_defaults"]["correction_limit"]),
            float(self.task["controller_defaults"]["feedback_low_pass_alpha"]),
            float(self.task["controller_defaults"]["feedback_average_window"])
        ], dtype=np.float32)
        return obs

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        self.last_action = action
        params = self._clip_params(action)
        metrics = self._evaluate(params)
        self.last_metrics = metrics
        self.eval_count += 1
        done = self.eval_count >= self.max_episode_evals
        self.current_sequence = self._sample_sequence()
        return self._observation(), float(metrics["reward"]), done, False, metrics
