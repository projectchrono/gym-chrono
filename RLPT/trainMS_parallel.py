import gym
import numpy as np
import torch
import os
from itertools import count

from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines.common import set_global_seeds
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
parser.add_argument('-a', '--arch', type=str, dest='arch', help='Actor Critic model type', default='MultiSensorLateFusion')
parser.add_argument('model_path', type=str, help='Where model should be saved to')
parser.add_argument('--onnx_converter',action='store_true', default=False, dest='onnx_converter', help='Toggle the onnx converter on')

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

if args.arch == 'MultiSensorLateFusion':
    from Model import MultiSensorLateFusion as ActorCritic
elif args.arch == 'MultiSensorEarlyFusion':
    from Model import MultiSensorEarlyFusion as ActorCritic
elif args.arch == 'MultiSensorSimple':
    from Model import MultiSensorSimple as ActorCritic
else:
    print('Model arch type not recognized. Exitting...')
    exit(1)

def make_env(env_id, rank, num_envs, seed=0):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environments you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = gym.make(env_id)
        env.rank = rank
        env.num_envs = num_envs
        env.seed(seed + rank)
        return env
    set_global_seeds(seed)
    return _init

if __name__ == "__main__":
    envs = SubprocVecEnv([make_env(env_name, i, num_envs) for i in range(num_envs)])

    img_size  = envs.observation_space[0].shape
    sensor_size  = envs.observation_space[1].shape
    num_outputs = envs.action_space.shape

    model = ActorCritic([img_size[1], img_size[0]], sensor_size[0], num_outputs[0]).to(device)
    if args.onnx_converter and os.path.isfile(modelpath):
        model.load_state_dict(torch.load(modelpath))

        model.export("gvsets_early_fusion.onnx")
        exit(1)

    if os.path.isfile(modelpath):
        model.load_state_dict(torch.load(modelpath))

    if not play_mode:
        ppo = PPO(model=model, envs=envs, device=device,  lr=lr, modelpath=modelpath, tuple_ob=True)
        ppo.ppo_train(num_steps, mini_batch_size, ppo_epochs,
                  max_frames, max_pol_updates,save_interval, increasing_length,
                  test_interval, savepath='./Monitor/')

    if not play_mode:
        exit()

    envs.set_attr('play_mode', True)
    state = envs.reset()
    done = np.zeros(num_envs)
    total_reward = np.zeros(num_envs)

    i_episode = np.zeros(num_envs)

    while True:
        arr_l = []
        for i in range(len(state)):
            arr = np.stack(state[i])
            arr_l.append(arr)

        state = [torch.FloatTensor(s).to(device) for s in arr_l]

        dist, _ = model(state)
        action = dist.mean.cpu().detach().numpy()
        next_state, reward, done, _ = envs.step(action)
        state = next_state
        total_reward += reward

        for i,temp_done in enumerate(done):
            if temp_done:
                print("episode:", i_episode[i], "reward:", total_reward[i], "steps:", num_steps)

                total_reward[i] = 0
                i_episode[i] += 1

                if sum(i_episode) > max_episodes:
                    print('Exceeded maximum of episodes.')
                    exit()

        print(str(total_reward))

        num_steps += 1
        # envs.render('human')
