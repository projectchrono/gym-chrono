import gymnasium as gym
from stable_baselines3 import PPO
from gym_chrono.envs.wheeled.off_road_gator import off_road_gator

import os

render = True
agent_render = True
if agent_render:
    env = off_road_gator(additional_render_mode='agent_pov')

else:
    env = off_road_gator()

i = 94
checkpoint_dir = '../train/gator_ppo_checkpoints'

loaded_model = PPO.load(os.path.join(
    checkpoint_dir, f"ppo_checkpoint{i}"), env)

sim_time = 20
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
