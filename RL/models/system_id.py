import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np


WHEEL_FIELDS = {
    "left": {
        "feedback": "left_filtered_count",
        "duty": "left_duty",
        "target": "left_target_count"
    },
    "right": {
        "feedback": "right_filtered_count",
        "duty": "right_duty",
        "target": "right_target_count"
    }
}


@dataclass
class PlantCoefficients:
    a1: float
    a2: float
    b1: float
    b2: float
    bias: float
    mse: float

    def to_dict(self) -> Dict[str, float]:
        return {
            "a1": self.a1,
            "a2": self.a2,
            "b1": self.b1,
            "b2": self.b2,
            "bias": self.bias,
            "mse": self.mse
        }


def _safe_float(value, default=0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def load_capture_frames(run_dir: Path) -> List[Dict]:
    status_path = run_dir / "status.json"
    if not status_path.exists():
        raise FileNotFoundError(f"missing capture status.json: {status_path}")
    payload = json.loads(status_path.read_text(encoding="utf-8"))
    return list(payload.get("statuses") or [])


def extract_series(frames: Iterable[Dict], wheel: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    names = WHEEL_FIELDS[wheel]
    feedback = []
    duty = []
    target = []
    for frame in frames:
      feedback.append(_safe_float(frame.get(names["feedback"])))
      duty.append(_safe_float(frame.get(names["duty"])))
      target.append(_safe_float(frame.get(names["target"])))
    return np.asarray(feedback, dtype=np.float64), np.asarray(duty, dtype=np.float64), np.asarray(target, dtype=np.float64)


def fit_arx(feedback: np.ndarray, duty: np.ndarray) -> PlantCoefficients:
    if feedback.size < 6 or duty.size < 6:
        raise ValueError("not enough samples for identification")

    rows = []
    targets = []
    for idx in range(2, feedback.size):
        rows.append([
            feedback[idx - 1],
            feedback[idx - 2],
            duty[idx - 1],
            duty[idx - 2],
            1.0
        ])
        targets.append(feedback[idx])

    x = np.asarray(rows, dtype=np.float64)
    y = np.asarray(targets, dtype=np.float64)
    coeffs, *_ = np.linalg.lstsq(x, y, rcond=None)
    preds = x @ coeffs
    mse = float(np.mean((preds - y) ** 2))
    return PlantCoefficients(
        a1=float(coeffs[0]),
        a2=float(coeffs[1]),
        b1=float(coeffs[2]),
        b2=float(coeffs[3]),
        bias=float(coeffs[4]),
        mse=mse
    )


def identify_from_run(run_dir: Path, task_name: str) -> Dict:
    frames = load_capture_frames(run_dir)
    result = {
        "task_name": task_name,
        "source_run": str(run_dir),
        "sample_count": len(frames),
        "wheels": {}
    }
    for wheel in ("left", "right"):
        feedback, duty, target = extract_series(frames, wheel)
        coeffs = fit_arx(feedback, duty)
        result["wheels"][wheel] = {
            "plant": coeffs.to_dict(),
            "series_summary": {
                "feedback_min": float(np.min(feedback)) if feedback.size else 0.0,
                "feedback_max": float(np.max(feedback)) if feedback.size else 0.0,
                "duty_min": float(np.min(duty)) if duty.size else 0.0,
                "duty_max": float(np.max(duty)) if duty.size else 0.0,
                "target_min": float(np.min(target)) if target.size else 0.0,
                "target_max": float(np.max(target)) if target.size else 0.0
            }
        }
    return result


def save_identification(model: Dict, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(model, indent=2), encoding="utf-8")


def load_identification(path: Path) -> Dict:
    return json.loads(path.read_text(encoding="utf-8"))


def latest_capture_run(raw_runs_dir: Path) -> Path:
    candidates = [path for path in raw_runs_dir.iterdir() if path.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"no capture runs under {raw_runs_dir}")
    candidates.sort(key=lambda item: item.stat().st_mtime, reverse=True)
    return candidates[0]


def ensure_identified_model(task_name: str, raw_runs_dir: Path, system_id_dir: Path, run_dir: Path = None) -> Path:
    run_path = run_dir or latest_capture_run(raw_runs_dir)
    output_path = system_id_dir / f"{task_name}.json"
    model = identify_from_run(run_path, task_name)
    save_identification(model, output_path)
    return output_path
