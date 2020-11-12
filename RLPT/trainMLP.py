import gym
import numpy as np
import torch
import os
from itertools import count

from baselines.common.vec_env import SubprocVecEnv
from baselines.common import set_global_seeds
from Model import ActorCriticPCoady as ActorCritic
from ppo import PPO

import argparse

parser = argparse.ArgumentParser(description=('Train GymChrono Environment'))
parser.add_argument('env_name', type=str, help='GymChrono environment name')
parser.add_argument('-s', '--num_steps', type=int, help='Number of steps to run', default=100)
parser.add_argument('-e', '--num_envs', type=int, help='Number of parallel environments to use', default=1)
parser.add_argument('-l', '--learning_rate', type=float, help='Learning rate', default=1e-4)
parser.add_argument('-m', '--mini_batch_size', type=int, help='Mini batch size', default=5)
parser.add_argument('-p', '--ppo_epochs', type=int, help='Number of PPO epochs', default=8)
parser.add_argument('-i', '--save_interval', type=int, help='Number of updates between saving', default=20)
parser.add_argument('-f', '--max_frames', type=float, help='Maximum number of frames', default=np.inf)
parser.add_argument('-u', '--max_updates', type=float, help='Maximum number of policy updates', default=500)
parser.add_argument('-t', '--test_interval', type=float, help='Interval at which mean rewards should be queried', default=20)
parser.add_argument('--increasing_length', type=float, help='Length at which num_steps should increase each after each update', default=0)
parser.add_argument('--play_mode',action='store_true', default=False, dest='play_mode', help='Toggle play mode on')
parser.add_argument('--max_episodes', type=int, help='Maximum episodes to run when in play_mode', default=1)
parser.add_argument('model_path', type=str, help='Where model should be saved to')

args = parser.parse_args()

env_name =          args.env_name
num_steps =         args.num_steps
num_envs =          args.num_envs
lr =                args.learning_rate
mini_batch_size  =  args.mini_batch_size
ppo_epochs =        args.ppo_epochs
save_interval =     args.save_interval
max_frames =        args.max_frames
max_pol_updates =   args.max_updates
test_interval =     args.test_interval
increasing_length = args.increasing_length
play_mode =         args.play_mode
max_episodes =      args.max_episodes
modelpath =         args.model_path
use_cuda =          torch.cuda.is_available()
device   =          torch.device("cuda" if use_cuda else "cpu")

def make_env(env_id, rank, seed=0):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environments you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = gym.make(env_id)
        env.seed(seed + rank)
        return env
    set_global_seeds(seed)
    return _init

if __name__ == "__main__":
    envs = SubprocVecEnv([make_env(env_name, i) for i in range(num_envs)])
    env = gym.make(env_name)

    num_inputs  = envs.observation_space.shape
    num_outputs = envs.action_space.shape
    if num_inputs == None:
        num_inputs = envs.observation_space[0].shape

    if len(num_inputs) == 3:
        num_inputs = list(num_inputs[:2])
    else:
        num_inputs = num_inputs[0]

    model = ActorCritic(num_inputs, num_outputs[0]).to(device)
    if os.path.isfile(modelpath):
        model.load_state_dict(torch.load(modelpath))

    ppo = PPO(model=model, envs=envs, device=device,  lr=lr, modelpath=modelpath)
    if not play_mode:
        ppo.ppo_train(num_steps, mini_batch_size, ppo_epochs,
                  max_frames, max_pol_updates,save_interval, increasing_length,
                  test_interval, savepath='./Monitor/')


    # <h1>Saving trajectories for GAIL</h1>
    max_expert_num = 50000
    num_steps = 0
    #expert_traj = []

    for i_episode in count():
        if not play_mode:
            break
        env.play_mode = True
        state = env.reset()
        done = False
        total_reward = 0

        while not done:
            state = torch.FloatTensor(state).unsqueeze(0).to(device)
            dist, _ = model(state)
            action = dist.sample().cpu().numpy()[0]
            next_state, reward, done, _ = env.step(action)
            state = next_state
            total_reward += reward
            print(str(total_reward))
            #expert_traj.append(np.hstack([state, action]))
            num_steps += 1
            env.render()

        print("episode:", i_episode, "reward:", total_reward, "steps:", num_steps)
        if i_episode >= max_episodes-1:
            break
        if num_steps >= max_expert_num:
            break
"""
    if play_mode:

        expert_traj = np.stack(expert_traj)
        print()
        print(expert_traj.shape)
        print()
        np.save("expert_traj.npy", expert_traj)
"""
