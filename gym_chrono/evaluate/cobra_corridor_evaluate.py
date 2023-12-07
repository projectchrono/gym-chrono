import gymnasium as gym
from stable_baselines3 import PPO
from gym_chrono.envs.wheeled.cobra_corridor import cobra_corridor
from gym_chrono.envs.utils.utils import chVector_to_npArray

import os

env = cobra_corridor()

checkpoint_dir = '../envs/data/trained_models/'

loaded_model = PPO.load(os.path.join(
    checkpoint_dir, f"cobra_corridor_example"), env)

sim_time = 50
timeStep = 0.1

totalSteps = int(sim_time / timeStep) + 1

obs, _ = env.reset(seed=0)
for step in range(totalSteps):
    action, _states = loaded_model.predict(obs, deterministic=True)
    print(f"Step {step + 1}")
    print("Action: ", action)
    obs, reward, teriminated, truncated, info = env.step(action)
    print("obs=", obs, "reward=", reward, "done=", (teriminated or truncated))
    env.render()
    if (teriminated or truncated):
        print("Goal is at: ", env.goal)
        print("Final position is: ", chVector_to_npArray(
            env.rover.GetChassis().GetPos()))
        print("Distance is: ", env._old_distance)
        print("Cumulative reward is: ", env._debug_reward)
        obs, _ = env.reset(seed=0)
        break
