import gymnasium as gym
from stable_baselines3 import PPO
from gym_chrono.envs.wheeled.off_road_art_lidar import off_road_art

import os

render = True
agent_render = True
if agent_render:
    env = off_road_art(additional_render_mode='agent_pov')

else:
    env = off_road_art()

checkpoint_dir = '../train/art_ppo_checkpoints_lidar_2/'

loaded_model = PPO.load(os.path.join(
    checkpoint_dir, f"ppo_checkpoint99"), env)

sim_time = 40
timeStep = 0.1

totalSteps = int(sim_time / timeStep)

env.set_nice_vehicle_mesh()
obs, _ = env.reset(seed=0)
if render:
    env.render('follow')
for step in range(totalSteps):
    action, _states = loaded_model.predict(obs, deterministic=True)
    print(f"Step {step + 1}")
    print("Action: ", action)
    obs, reward, teriminated, truncated, info = env.step(action)
    print("obs=", obs, "reward=", reward, "done=", (teriminated or truncated))
    env.render('follow')
    if (teriminated or truncated):
        break
