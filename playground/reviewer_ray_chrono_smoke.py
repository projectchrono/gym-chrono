#!/usr/bin/env python3
"""
Minimal Ray + ChronoViperWalk smoke for AMD Linux (Instinct + ROCm).

Prerequisites:
  - CHRONO_DATA_DIR, PYTHONPATH includes gym-chrono and PyChrono build
  - ray start --head --num-gpus=N (optional if you only test CPU actors)
  - RAY_EXPERIMENTAL_NOSET_* set before import ray (this script does that)

Run:
  python3 playground/reviewer_ray_chrono_smoke.py
"""
from __future__ import annotations

import os

os.environ.setdefault("RAY_EXPERIMENTAL_NOSET_HIP_VISIBLE_DEVICES", "1")
os.environ.setdefault("RAY_EXPERIMENTAL_NOSET_ROCR_VISIBLE_DEVICES", "1")

import ray

_CHRONO_CPU = {
    "env_vars": {
        "HIP_VISIBLE_DEVICES": "-1",
        "ROCR_VISIBLE_DEVICES": "-1",
    }
}


@ray.remote(num_cpus=1, runtime_env=_CHRONO_CPU)
class ViperActor:
    def __init__(self, seed: int) -> None:
        import gymnasium as gym

        import gym_chrono  # noqa: F401 — registration

        self._env = gym.make("ChronoViperWalk-v0", render_mode=None)
        self._env.reset(seed=seed)

    def step(self) -> float:
        a = self._env.action_space.sample()
        _obs, r, _term, _trunc, _info = self._env.step(a)
        return float(r)


def main() -> None:
    if not os.environ.get("CHRONO_DATA_DIR"):
        raise SystemExit("Set CHRONO_DATA_DIR to your Chrono data directory.")
    ray.init()
    actors = [ViperActor.remote(i) for i in range(2)]
    rewards = ray.get([a.step.remote() for a in actors])
    print("rewards", rewards)
    ray.shutdown()


if __name__ == "__main__":
    main()
