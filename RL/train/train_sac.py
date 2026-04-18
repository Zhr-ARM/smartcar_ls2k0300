import argparse
import json
import random
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from RL.envs.speed_pid_env import PARAM_ORDER, SpeedPidEnv  # noqa: E402
from RL.models.system_id import ensure_identified_model  # noqa: E402

try:
    from stable_baselines3 import SAC
    SB3_AVAILABLE = True
except Exception:
    SAC = None
    SB3_AVAILABLE = False


def write_status(path: Path, payload: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload["updated_at_ms"] = int(time.time() * 1000)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def export_best(output_dir: Path, task_name: str, algorithm: str, score: float, params: Dict, model_path: Path, task_config: Path) -> Dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    best_path = output_dir / "best_params.json"
    payload = {
        "task_name": task_name,
        "algorithm": algorithm,
        "score": float(score),
        "params": params,
        "model_path": str(model_path),
        "task_config": str(task_config)
    }
    best_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def run_random_search(env: SpeedPidEnv, task: Dict, result_dir: Path, model_path: Path, task_config: Path, status_path: Path) -> Dict:
    bounds = task["parameter_bounds"]
    rng = np.random.default_rng(int(task.get("seed", 42)))
    best_score = -1e18
    best_params = None
    trials = int(task["training"].get("random_search_trials", 600))

    for trial in range(trials):
        action = np.asarray([
            rng.uniform(bounds[key][0], bounds[key][1]) for key in PARAM_ORDER
        ], dtype=np.float32)
        env.reset(seed=trial + 1)
        _, reward, _, _, info = env.step(action)
        if reward > best_score:
            best_score = float(reward)
            best_params = {key: float(action[idx]) for idx, key in enumerate(PARAM_ORDER)}
            export_best(result_dir, task["task_name"], "random_search_fallback", best_score, best_params, model_path, task_config)
        if trial % 20 == 0 or trial == trials - 1:
            write_status(status_path, {
                "ok": True,
                "phase": "training",
                "algorithm": "random_search_fallback",
                "progress": trial + 1,
                "total": trials,
                "best_reward": best_score,
                "best_params": best_params,
                "last_metrics": info
            })

    return {"score": best_score, "params": best_params, "algorithm": "random_search_fallback"}


def run_sac(env: SpeedPidEnv, task: Dict, result_dir: Path, model_path: Path, task_config: Path, status_path: Path) -> Dict:
    cfg = task["training"]
    model = SAC(
        "MlpPolicy",
        env,
        verbose=0,
        learning_starts=int(cfg.get("learning_starts", 256)),
        batch_size=int(cfg.get("batch_size", 64)),
        buffer_size=int(cfg.get("buffer_size", 20000)),
        seed=int(task.get("seed", 42))
    )
    total_timesteps = int(cfg.get("total_timesteps", 4000))
    chunk = min(500, total_timesteps)
    trained = 0
    best_reward = -1e18
    best_params = None

    while trained < total_timesteps:
        step_count = min(chunk, total_timesteps - trained)
        model.learn(total_timesteps=step_count, reset_num_timesteps=False, progress_bar=False)
        trained += step_count

        for eval_seed in range(8):
            obs, _ = env.reset(seed=eval_seed + trained)
            action, _ = model.predict(obs, deterministic=True)
            _, reward, _, _, info = env.step(action)
            if reward > best_reward:
                best_reward = float(reward)
                best_params = {key: float(action[idx]) for idx, key in enumerate(PARAM_ORDER)}
                export_best(result_dir, task["task_name"], "sac", best_reward, best_params, model_path, task_config)

        write_status(status_path, {
            "ok": True,
            "phase": "training",
            "algorithm": "sac",
            "progress": trained,
            "total": total_timesteps,
            "best_reward": best_reward,
            "best_params": best_params
        })

    return {"score": best_reward, "params": best_params, "algorithm": "sac"}


def run_evaluation(task_config: Path, model_path: Path, best_params_path: Path, result_dir: Path) -> None:
    subprocess.run([
        sys.executable,
        str(ROOT / "RL" / "eval" / "evaluate_policy.py"),
        "--task-config", str(task_config),
        "--model-path", str(model_path),
        "--best-params", str(best_params_path),
        "--output", str(result_dir / "evaluation.json"),
        "--episodes", "24"
    ], check=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-config", required=True)
    parser.add_argument("--capture-run")
    parser.add_argument("--status-path")
    args = parser.parse_args()

    task_config = Path(args.task_config).resolve()
    task = json.loads(task_config.read_text(encoding="utf-8"))
    task_name = task["task_name"]
    random.seed(int(task.get("seed", 42)))
    np.random.seed(int(task.get("seed", 42)))

    raw_runs_dir = ROOT / "RL" / "data" / "raw_runs"
    system_id_dir = ROOT / "RL" / "data" / "system_id"
    result_dir = ROOT / "RL" / "results" / task_name
    status_path = Path(args.status_path).resolve() if args.status_path else (ROOT / "RL" / "data" / "train_status" / f"{task_name}.json")

    write_status(status_path, {
        "ok": True,
        "phase": "identifying",
        "task_name": task_name
    })

    run_dir = Path(args.capture_run).resolve() if args.capture_run else None
    model_path = ensure_identified_model(task_name, raw_runs_dir, system_id_dir, run_dir)
    env = SpeedPidEnv(str(task_config), str(model_path), seed=int(task.get("seed", 42)))

    if SB3_AVAILABLE:
        summary = run_sac(env, task, result_dir, model_path, task_config, status_path)
    else:
        summary = run_random_search(env, task, result_dir, model_path, task_config, status_path)

    best_path = result_dir / "best_params.json"
    if best_path.exists():
        run_evaluation(task_config, model_path, best_path, result_dir)

    write_status(status_path, {
        "ok": True,
        "phase": "completed",
        "task_name": task_name,
        "algorithm": summary["algorithm"],
        "best_reward": summary["score"],
        "best_params": summary["params"],
        "result_dir": str(result_dir),
        "model_path": str(model_path)
    })
    print(json.dumps({
        "ok": True,
        "task_name": task_name,
        "algorithm": summary["algorithm"],
        "best_reward": summary["score"],
        "result_dir": str(result_dir)
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
