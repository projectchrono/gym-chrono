#!/usr/bin/env python3
"""Smoke test for ChronoViperWalk-v0 (WS3a). Run from repo root:

  export CHRONO_DATA_DIR=/path/to/chrono/data/
  export PYTHONPATH=/path/to/gym-chrono-ws3
  python3 gym_chrono/test/smoke_viper_walk.py
"""
from __future__ import annotations

import os
import sys


def main() -> int:
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if root not in sys.path:
        sys.path.insert(0, root)

    import gymnasium as gym

    import gym_chrono  # noqa: F401 — registers envs

    env = gym.make("ChronoViperWalk-v0", render_mode=None)
    obs, _ = env.reset(seed=0)
    a = env.action_space.sample()
    obs2, r, term, trunc, _ = env.step(a)
    env.close()
    print("ok", obs.shape, obs2.shape, r, term, trunc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
