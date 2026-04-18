import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from RL.envs.speed_pid_env import PARAM_ORDER, SpeedPidEnv


def evaluate(task_config: Path, model_path: Path, best_params_path: Path, episodes: int) -> dict:
    env = SpeedPidEnv(str(task_config), str(model_path))
    payload = json.loads(best_params_path.read_text(encoding="utf-8"))
    params = payload["params"]
    action = np.asarray([float(params[key]) for key in PARAM_ORDER], dtype=np.float32)

    rewards = []
    main_errors = []
    tail_errors = []
    overshoots = []
    heats = []

    for episode_idx in range(episodes):
        env.reset(seed=episode_idx + 1)
        _, reward, _, _, info = env.step(action)
        rewards.append(float(reward))
        main_errors.append(float(info["main_error_sum"]))
        tail_errors.append(float(info["tail_error_sum"]))
        overshoots.append(float(info["overshoot_sum"]))
        heats.append(float(info["heat_penalty"]))

    return {
        "episodes": episodes,
        "reward_mean": float(np.mean(rewards)),
        "reward_std": float(np.std(rewards)),
        "main_error_mean": float(np.mean(main_errors)),
        "tail_error_mean": float(np.mean(tail_errors)),
        "overshoot_mean": float(np.mean(overshoots)),
        "heat_penalty_mean": float(np.mean(heats)),
        "params": params
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-config", required=True)
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--best-params", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--episodes", type=int, default=24)
    args = parser.parse_args()

    result = evaluate(Path(args.task_config), Path(args.model_path), Path(args.best_params), args.episodes)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"ok": True, "evaluation_path": str(output), "reward_mean": result["reward_mean"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
