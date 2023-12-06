import torch as th
from torch import nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy

import numpy as np
import gymnasium as gym
import copy


def outputSize(in_size, kernel_size, stride, padding):
    conv_size = [0, 0]
    for i in range(len(kernel_size)):
        conv_size[0] = int((conv_size[0] - kernel_size[i] +
                           2*(padding[i])) / stride[i]) + 1
        conv_size[1] = int((conv_size[1] - kernel_size[i] +
                           2*(padding[i])) / stride[i]) + 1

    return tuple(conv_size)


class CustomCombinedExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=768):
        super(CustomCombinedExtractor, self).__init__(
            observation_space, features_dim)
        n_input_channels = 3

        extractors = {}

        for key, space in observation_space.spaces.items():
            if key == "image":
                print(space)
                n_input_channels = 3
                features_dim = 10
                print(space.shape)
                fc_size = outputSize(space.shape, [8, 4, 3], [
                                     4, 2, 1], [0, 0, 0])
                extractors[key] = nn.Sequential(
                    nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4),
                    nn.ReLU(),
                    nn.Conv2d(32, 64, kernel_size=4, stride=2),
                    nn.ReLU(),
                    nn.Conv2d(64, 64, kernel_size=3, stride=1),
                    nn.ReLU(),
                    nn.Flatten(),
                    nn.Linear(768, features_dim),
                    nn.ReLU()
                )
            else:
                extractors[key] = nn.Sequential(
                    # Assuming the additional features are a flat vector
                    nn.Linear(5, 10),
                    nn.ReLU()
                )
        self.extractors = nn.ModuleDict(extractors)
        self._features_dim = 20

    def forward(self, observations):
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)
