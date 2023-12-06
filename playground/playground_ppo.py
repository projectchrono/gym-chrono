import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
import os
import torch

def make_env(env_name, rank, seed=0):
    def _init():
        env = gym.make(env_name)
        if hasattr(env, 'seed'):
            env.seed(seed + rank)
        elif hasattr(env.unwrapped, 'seed'):
            env.unwrapped.seed(seed + rank)
        return env
    return _init

if __name__ == '__main__':
    num_cpu = 20  # Number of processes to use
    env_name = "HumanoidStandup-v4"

    env = SubprocVecEnv([make_env(env_name, i) for i in range(num_cpu)])

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Using device:", device)

    model = PPO('MlpPolicy', env, learning_rate=5e-4, verbose=1, device=device)

    checkpoint_dir = 'ppo_checkpoints'
    os.makedirs(checkpoint_dir, exist_ok=True)

    render_env = gym.make(env_name)  # Separate environment for rendering
    render_interval = 1000  # Render every 1000 iterations

    for i in range(100000):
        model.learn(300000)
        model.save(os.path.join(checkpoint_dir, f"ppo_checkpoint{i}"))

        # Render the environment at intervals
        if i % render_interval == 0:
            obs = render_env.reset()
            for _ in range(1000):  # Adjust this for longer/shorter renderings
                action, _ = model.predict(obs)
                obs, _, done, _ = render_env.step(action)
                render_env.render()
                if done:
                    obs = render_env.reset()

    env.close()
    render_env.close()
