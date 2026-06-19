import logging

import gymnasium as gym
from gymnasium.envs.registration import register

register(
    id="art_wpts-v0",
    entry_point="gym_chrono.envs:art_wpts",
)  # legacy registration (may be unused — preserved for compatibility)

register(
    id="ChronoViperWalk-v0",
    entry_point="gym_chrono.envs.legged.viper_walk:ChronoViperWalkEnv",
    max_episode_steps=100_000,
)
