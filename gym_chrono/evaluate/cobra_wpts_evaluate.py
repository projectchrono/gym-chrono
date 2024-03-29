import gymnasium as gym
from stable_baselines3 import A2C, SAC, PPO, TD3
from gym_chrono.envs.wheeled.cobra_wpts import cobra_wpts
from stable_baselines3.common.evaluation import evaluate_policy
from gym_chrono.envs.utils.utils import CalcInitialPose, chVector_to_npArray, SetChronoDataDirectories

import os

env = cobra_wpts()

checkpoint_dir = '../envs/data/trained_models/'

loaded_model = PPO.load(os.path.join(
    checkpoint_dir, f"cobra_wpts_example"), env)


sim_time = 180
timeStep = 0.2

totalSteps = int(sim_time / timeStep)

obs, _ = env.reset(seed=0)
for step in range(totalSteps):
    action, _states = loaded_model.predict(obs, deterministic=True)
    print(f"Step {step + 1}")
    print("Action: ", action)
    obs, reward, teriminated, truncated, info = env.step(action)
    print("obs=", obs, "reward=", reward, "done=", (teriminated or truncated))
    env.render()
    if (teriminated or truncated):
        obs, _ = env.reset(seed=0)
        break
