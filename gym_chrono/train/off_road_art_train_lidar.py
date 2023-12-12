# =======================================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =======================================================================================
# Authors: Huzaifa Unjhawala
# =======================================================================================
#
# This file contains a script to train a PPO Cobra agent to navigate to a goal point
# A Tensorboard is used for logging of training statistics. The training statistics are
# saved in the logs folder.  Checkpoints are saved in the ppo_checkpoints folder.
#
# =======================================================================================
import gymnasium as gym

from typing import Callable
import os

from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import HParam
import torch as th


from gym_chrono.envs.wheeled.off_road_art_lidar import off_road_art
from gym_chrono.train.custom_networks.artCustomLidar import CustomCombinedExtractor


class TensorboardCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """

    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)
        self.new_success_rate = 0.0
        self.old_episode_num = 0
        self.old_timeout_count = 0
        self.old_crash_count = 0
        self.old_fallen_count = 0
        self.old_success_count = 0

    def _on_training_start(self) -> None:
        hparam_dict = {
            "algorithm": self.model.__class__.__name__,
            "learning rate": self.model.learning_rate,
            "gamma": self.model.gamma,
        }
        # define the metrics that will appear in the `HPARAMS` Tensorboard tab by referencing their tag
        # Tensorbaord will find & display metrics from the `SCALARS` tab
        metric_dict = {
            "rollout/ep_rew_mean": 0,
            "train/value_loss": 0.0,
        }
        self.logger.record(
            "hparams",
            HParam(hparam_dict, metric_dict),
            exclude=("stdout", "log", "json", "csv"),
        )

    def _on_rollout_start(self) -> None:
        # Reset all the tracking numbers
        # zeros_list = [0] * num_cpu
        # self.training_env.set_attr("m_episode_num", 0)
        # self.training_env.set_attr("m_num_obstacles", 0)
        # self.training_env.set_attr("m_success_count", 0)
        # self.training_env.set_attr("m_crash_count", 0)
        # self.training_env.set_attr("m_fallen_count", 0)
        # self.training_env.set_attr("m_timeout_count", 0)
        return True

    def _on_rollout_end(self) -> None:
        # Aggregate data from all environments
        total_success_count = sum(
            self.training_env.get_attr("m_success_count"))
        total_crash_count = sum(self.training_env.get_attr("m_crash_count"))
        total_fallen_count = sum(self.training_env.get_attr("m_fallen_count"))
        total_timeout_count = sum(
            self.training_env.get_attr("m_timeout_count"))
        total_num_obstacles = sum(
            self.training_env.get_attr("m_num_obstacles"))
        total_episode_num = sum(self.training_env.get_attr("m_episode_num"))

        # Log the rates
        self.logger.record("rollout/total_success", total_success_count)
        self.logger.record("rollout/total_crashes", total_crash_count)
        self.logger.record("rollout/total_fallen", total_fallen_count)
        self.logger.record("rollout/total_timeout", total_timeout_count)
        self.logger.record("rollout/obstacles_per_reset",
                           total_num_obstacles/num_cpu)
        self.logger.record("rollout/total_episode_num", total_episode_num)

        self.old_episode_num = total_episode_num
        self.old_timeout_count = total_timeout_count
        self.old_crash_count = total_crash_count
        self.old_fallen_count = total_fallen_count
        self.old_success_count = total_success_count
        return True

    def _on_training_end(self) -> None:
        print("Training ended")
        print("Total episodes ran: ", self.old_episode_num)
        print("Total success count: ", self.old_success_count)
        print("Total crash count: ", self.old_crash_count)
        print("Total fallen count: ", self.old_fallen_count)
        print("Total timeout count: ", self.old_timeout_count)
        return True

    def _on_step(self) -> bool:
        return True


def make_env(rank: int, seed: int = 0) -> Callable:
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environment you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    :return: (Callable)
    """

    def _init() -> gym.Env:
        env = off_road_art()
        env.set_nice_vehicle_mesh()
        env.reset(seed=seed + rank)
        return env

    set_random_seed(seed)
    return _init


if __name__ == '__main__':
    env_single = off_road_art()
    num_cpu = 6
    # Set to make an update after the end of 2 episodes (30 s each)
    n_steps = 40 * 2 * 10
    # Set mini batch is the experiences so that 1/2th  batch is consumed to make an update
    batch_size = n_steps // 2

    # Set the number of timesteps such that we get 200 updates
    num_of_updates = 200
    total_timesteps = num_of_updates * n_steps * num_cpu

    log_path = "art_logs_lidar_2/"
    # set up logger
    new_logger = configure(log_path, ["stdout", "csv", "tensorboard"])
    # Vectorized envieroment
    env = make_vec_env(env_id=make_env(0), n_envs=num_cpu,
                       vec_env_cls=SubprocVecEnv)
    policy_kwargs = dict(
        features_extractor_class=CustomCombinedExtractor,
        features_extractor_kwargs={'features_dim': 10},
        activation_fn=th.nn.ReLU,
        net_arch=dict(activation_fn=th.nn.ReLU, pi=[40, 20, 10], vf=[40, 20, 10]))

    model = PPO('MultiInputPolicy', env, learning_rate=5e-4, n_steps=n_steps,
                batch_size=batch_size, policy_kwargs=policy_kwargs, verbose=1, n_epochs=10,  tensorboard_log=log_path)

    print(model.policy)
    model.set_logger(new_logger)
    reward_store = []
    std_reward_store = []
    # Get total 100 saves - every 2 updates we save the model
    num_of_saves = num_of_updates // 2
    training_steps_per_save = total_timesteps // num_of_saves
    checkpoint_dir = 'art_ppo_checkpoints_lidar_2'
    os.makedirs(checkpoint_dir, exist_ok=True)

    # In case user wants to load from a certain checkpoint
    model = PPO.load(os.path.join(
        checkpoint_dir, f"ppo_checkpoint55"), env)
    # Replace the max range to num_of_saves ideally but the memory of my env keeps ballooning up
    success_rate_eval = 0.
    for i in range(56, 100):
        env.env_method("set_succ", success_rate_eval)
        success_rate_eval = 0.
        model.learn(training_steps_per_save, callback=TensorboardCallback())
        model.save(os.path.join(checkpoint_dir, f"ppo_checkpoint{i}"))
        model = PPO.load(os.path.join(
            checkpoint_dir, f"ppo_checkpoint{i}"), env)

        # Evaluate the loaded model every 10 updates - saves are made every 2 updates
        if (i % 5 == 0):
            sucess_count = 0
            for _ in range(0, 10):
                obs, some = env_single.reset()
                done = False
                while not done:
                    action, _states = model.predict(obs, deterministic=True)
                    obs, rewards, teriminated, truncated, info = env_single.step(
                        action)
                    done = teriminated or truncated

                sucess_count += env_single.m_success_count_eval

            print("Sucess rate of loaded model: ", sucess_count/10)
            success_rate_eval = sucess_count/10
            if (success_rate_eval > 0.6):
                # Write to a file that the model with checkpoint i is good
                f = open("./" + log_path + "good_models.txt", "a")
                f.write(f"ppo_checkpoint{i}\n")
                f.close()
