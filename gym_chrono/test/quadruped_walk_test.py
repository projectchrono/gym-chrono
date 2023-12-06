import gymnasium as gym


from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

import numpy as np
import math


from gym_chrono.envs.legged.quadruped_walk import quadruped_walk

if __name__ == '__main__':
    env = quadruped_walk()
    check_env(env)

    obs, _ = env.reset()
    env.render()

    print(env.observation_space)
    print(env.action_space)
    print(env.action_space.sample())

    # Hardcoded best agent: always go left!
    n_steps = 1000000
    for step in range(n_steps):
        print(f"Step {step + 1}")
        # obs, reward, terminated, truncated, info = env.step(
        #     env.action_space.sample())
        obs, reward, terminated, truncated, info = env.step(np.linspace(-math.pi, math.pi, 12))
        done = terminated or truncated
        print("obs=", obs, "reward=", reward, "done=", done)
        env.render()
        if done:
            print("reward=", reward)
            break
