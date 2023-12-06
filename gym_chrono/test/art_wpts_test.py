import gymnasium as gym
from stable_baselines3 import A2C, SAC, PPO, TD3
from gym_chrono.envs.wheeled.art_wpts import art_wpts

env = art_wpts()

loaded_model = PPO.load("PPO_tutorial")


obs, _ = env.reset(seed=0)
for step in range(10000):
    action, _states = loaded_model.predict(obs, deterministic=True)
    print(f"Step {step + 1}")
    print("Action: ", action)
    obs, reward, teriminated, truncated, info = env.step(action)
    print("obs=", obs, "reward=", reward, "done=", (teriminated or truncated))
    env.render()
    if (teriminated or truncated):
        # Note that the VecEnv resets automatically
        # when a done signal is encountered
        print("Goal reached!", "reward=", reward)
        obs, _ = env.reset(seed=0)
        # break
