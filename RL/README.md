# RL Speed PID Tuning

This directory contains the first-pass reinforcement learning workflow for
motor speed-loop PID tuning.

## Goals

- Collect step-response logs from the real board through the existing web/config
  pipeline.
- Identify left/right wheel discrete plant models from recorded logs.
- Train a continuous-action policy for the six core parameters:
  - `left_kp`
  - `left_ki`
  - `left_feedforward_gain`
  - `right_kp`
  - `right_ki`
  - `right_feedforward_gain`
- Export the best parameters back to `project/user/smartcar_config.toml`.

## Layout

- `configs/`: task configs, bounds, reward weights, capture templates
- `data/raw_runs/`: captured board logs
- `data/system_id/`: fitted plant models
- `data/train_status/`: live training status files written by Python
- `results/`: exported best parameters and evaluation summaries
- `envs/`: training environment
- `models/`: system identification and plant simulation logic
- `train/`: training entrypoint
- `eval/`: evaluation entrypoint

## Main Flow

1. Use the RL page in `tools/pc_receiver_js` to run automatic step capture.
2. Captured logs are written under `RL/data/raw_runs/<capture_id>/`.
3. Start training from the same page or run:

```bash
python3 RL/train/train_sac.py --task-config RL/configs/task_speed_pid_default.json
```

4. The trainer:
   - reads the latest capture
   - fits or reloads a plant model
   - trains with SAC when `stable-baselines3` is available
   - falls back to bounded random search if the RL library is unavailable

5. Best results are written to:

```text
RL/results/<task_name>/best_params.json
RL/results/<task_name>/evaluation.json
```

## Python Dependencies

Preferred:

- `numpy`
- `gymnasium`
- `stable-baselines3`

Fallback mode:

- if `stable-baselines3` or `gymnasium` is missing, the training script still
  runs with a bounded search fallback so the full pipeline remains testable

## Notes

- The first version only tunes the six core parameters listed above.
- The environment optimizes:
  - first 2-frame absolute error sum
  - small-weight 20-frame absolute error sum
  - overshoot and heat-risk proxy penalties
- The real board is used only for capture and final validation, not for online
  exploration.
