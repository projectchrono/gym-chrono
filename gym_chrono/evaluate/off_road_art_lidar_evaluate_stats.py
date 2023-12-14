import gymnasium as gym
from stable_baselines3 import PPO
from gym_chrono.envs.wheeled.off_road_art_lidar_evaluate import off_road_art
import os
import numpy as np


sucess_count = 0
episode_reward_list = []
env_single = off_road_art()
epi_to_run = 100

deterministic = True

checkpoint_dir = '../train/art_ppo_checkpoints_lidar_4/'

checkpoint = "ppo_checkpoint100"

loaded_model = PPO.load(os.path.join(
    checkpoint_dir, checkpoint), env_single)

for _ in range(0, epi_to_run):
    Accumulated_reward = 0
    obs, some = env_single.reset()
    done = False
    while not done:
        if (deterministic):
            action, _states = loaded_model.predict(
                obs, deterministic=True)
        else:
            action, _states = loaded_model.predict(
                obs, deterministic=False)
        obs, rewards, teriminated, truncated, info = env_single.step(
            action)
        Accumulated_reward += rewards
        done = teriminated or truncated
    episode_reward_list.append(Accumulated_reward)
    sucess_count += env_single.m_success_count_eval

print("Sucess rate of loaded model: ", sucess_count/epi_to_run)
mean_reward = np.mean(episode_reward_list)
std_reward = np.std(episode_reward_list)
print("Mean reward of loaded model: ", mean_reward)
print("standard deviation of reward of loaded model: ", std_reward)

# Write these to a file
if (deterministic):
    f = open("./eval_rewards_" + checkpoint + ".txt", "a")
else:
    f = open("./eval_rewards_" + checkpoint + "_stochastic.txt", "a")

f.write(f"Sucess rate of loaded model: {sucess_count/epi_to_run}\n")
f.write(
    f"Mean reward of loaded model: {np.mean(episode_reward_list)}\n")
f.write(
    f"standard deviation of reward of loaded model: {np.std(episode_reward_list)}\n")
f.close()
